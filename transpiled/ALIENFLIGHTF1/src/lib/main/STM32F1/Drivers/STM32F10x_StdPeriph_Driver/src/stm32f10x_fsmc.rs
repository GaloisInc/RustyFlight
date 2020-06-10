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
  * @brief Flexible Static Memory Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FSMC_Bank1_TypeDef {
    pub BTCR: [uint32_t; 8],
}
/* * 
  * @brief Flexible Static Memory Controller Bank1E
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FSMC_Bank1E_TypeDef {
    pub BWTR: [uint32_t; 7],
}
/* * 
  * @brief Flexible Static Memory Controller Bank2
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FSMC_Bank2_TypeDef {
    pub PCR2: uint32_t,
    pub SR2: uint32_t,
    pub PMEM2: uint32_t,
    pub PATT2: uint32_t,
    pub RESERVED0: uint32_t,
    pub ECCR2: uint32_t,
}
/* * 
  * @brief Flexible Static Memory Controller Bank3
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FSMC_Bank3_TypeDef {
    pub PCR3: uint32_t,
    pub SR3: uint32_t,
    pub PMEM3: uint32_t,
    pub PATT3: uint32_t,
    pub RESERVED0: uint32_t,
    pub ECCR3: uint32_t,
}
/* * 
  * @brief Flexible Static Memory Controller Bank4
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FSMC_Bank4_TypeDef {
    pub PCR4: uint32_t,
    pub SR4: uint32_t,
    pub PMEM4: uint32_t,
    pub PATT4: uint32_t,
    pub PIO4: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FSMC_NORSRAMTimingInitTypeDef {
    pub FSMC_AddressSetupTime: uint32_t,
    pub FSMC_AddressHoldTime: uint32_t,
    pub FSMC_DataSetupTime: uint32_t,
    pub FSMC_BusTurnAroundDuration: uint32_t,
    pub FSMC_CLKDivision: uint32_t,
    pub FSMC_DataLatency: uint32_t,
    pub FSMC_AccessMode: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FSMC_NORSRAMInitTypeDef {
    pub FSMC_Bank: uint32_t,
    pub FSMC_DataAddressMux: uint32_t,
    pub FSMC_MemoryType: uint32_t,
    pub FSMC_MemoryDataWidth: uint32_t,
    pub FSMC_BurstAccessMode: uint32_t,
    pub FSMC_AsynchronousWait: uint32_t,
    pub FSMC_WaitSignalPolarity: uint32_t,
    pub FSMC_WrapMode: uint32_t,
    pub FSMC_WaitSignalActive: uint32_t,
    pub FSMC_WriteOperation: uint32_t,
    pub FSMC_WaitSignal: uint32_t,
    pub FSMC_ExtendedMode: uint32_t,
    pub FSMC_WriteBurst: uint32_t,
    pub FSMC_ReadWriteTimingStruct: *mut FSMC_NORSRAMTimingInitTypeDef,
    pub FSMC_WriteTimingStruct: *mut FSMC_NORSRAMTimingInitTypeDef,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FSMC_NAND_PCCARDTimingInitTypeDef {
    pub FSMC_SetupTime: uint32_t,
    pub FSMC_WaitSetupTime: uint32_t,
    pub FSMC_HoldSetupTime: uint32_t,
    pub FSMC_HiZSetupTime: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FSMC_NANDInitTypeDef {
    pub FSMC_Bank: uint32_t,
    pub FSMC_Waitfeature: uint32_t,
    pub FSMC_MemoryDataWidth: uint32_t,
    pub FSMC_ECC: uint32_t,
    pub FSMC_ECCPageSize: uint32_t,
    pub FSMC_TCLRSetupTime: uint32_t,
    pub FSMC_TARSetupTime: uint32_t,
    pub FSMC_CommonSpaceTimingStruct: *mut FSMC_NAND_PCCARDTimingInitTypeDef,
    pub FSMC_AttributeSpaceTimingStruct: *mut FSMC_NAND_PCCARDTimingInitTypeDef,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FSMC_PCCARDInitTypeDef {
    pub FSMC_Waitfeature: uint32_t,
    pub FSMC_TCLRSetupTime: uint32_t,
    pub FSMC_TARSetupTime: uint32_t,
    pub FSMC_CommonSpaceTimingStruct: *mut FSMC_NAND_PCCARDTimingInitTypeDef,
    pub FSMC_AttributeSpaceTimingStruct: *mut FSMC_NAND_PCCARDTimingInitTypeDef,
    pub FSMC_IOSpaceTimingStruct: *mut FSMC_NAND_PCCARDTimingInitTypeDef,
}
/* *
  * @}
  */
/* * @defgroup FSMC_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the FSMC NOR/SRAM Banks registers to their default 
  *         reset values.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank1_NORSRAM1: FSMC Bank1 NOR/SRAM1  
  *     @arg FSMC_Bank1_NORSRAM2: FSMC Bank1 NOR/SRAM2 
  *     @arg FSMC_Bank1_NORSRAM3: FSMC Bank1 NOR/SRAM3 
  *     @arg FSMC_Bank1_NORSRAM4: FSMC Bank1 NOR/SRAM4 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_NORSRAMDeInit(mut FSMC_Bank: uint32_t) {
    /* Check the parameter */
    /* FSMC_Bank1_NORSRAM1 */
    if FSMC_Bank == 0 as libc::c_int as uint32_t {
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank1_TypeDef)).BTCR[FSMC_Bank
                                                                                   as
                                                                                   usize]
                                        as *mut uint32_t,
                                    0x30db as libc::c_int as uint32_t)
    } else {
        /* FSMC_Bank1_NORSRAM2,  FSMC_Bank1_NORSRAM3 or FSMC_Bank1_NORSRAM4 */
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank1_TypeDef)).BTCR[FSMC_Bank
                                                                                   as
                                                                                   usize]
                                        as *mut uint32_t,
                                    0x30d2 as libc::c_int as uint32_t)
    }
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as
                                            *mut FSMC_Bank1_TypeDef)).BTCR[FSMC_Bank.wrapping_add(1
                                                                                                      as
                                                                                                      libc::c_int
                                                                                                      as
                                                                                                      libc::c_uint)
                                                                               as
                                                                               usize]
                                    as *mut uint32_t,
                                0xfffffff as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0x104
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as
                                            *mut FSMC_Bank1E_TypeDef)).BWTR[FSMC_Bank
                                                                                as
                                                                                usize]
                                    as *mut uint32_t,
                                0xfffffff as libc::c_int as uint32_t);
}
/* *
  * @brief  Deinitializes the FSMC NAND Banks registers to their default reset values.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank2_NAND: FSMC Bank2 NAND 
  *     @arg FSMC_Bank3_NAND: FSMC Bank3 NAND 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_NANDDeInit(mut FSMC_Bank: uint32_t) {
    /* Check the parameter */
    if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
        /* Set the FSMC_Bank2 registers to their reset values */
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x60
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank2_TypeDef)).PCR2
                                        as *mut uint32_t,
                                    0x18 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x60
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank2_TypeDef)).SR2
                                        as *mut uint32_t,
                                    0x40 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x60
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank2_TypeDef)).PMEM2
                                        as *mut uint32_t,
                                    0xfcfcfcfc as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x60
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank2_TypeDef)).PATT2
                                        as *mut uint32_t,
                                    0xfcfcfcfc as libc::c_uint)
    } else {
        /* FSMC_Bank3_NAND */
        /* Set the FSMC_Bank3 registers to their reset values */
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x80
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank3_TypeDef)).PCR3
                                        as *mut uint32_t,
                                    0x18 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x80
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank3_TypeDef)).SR3
                                        as *mut uint32_t,
                                    0x40 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x80
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank3_TypeDef)).PMEM3
                                        as *mut uint32_t,
                                    0xfcfcfcfc as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x80
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank3_TypeDef)).PATT3
                                        as *mut uint32_t,
                                    0xfcfcfcfc as libc::c_uint)
    };
}
/* *
  * @brief  Deinitializes the FSMC PCCARD Bank registers to their default reset values.
  * @param  None                       
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_PCCARDDeInit() {
    /* Set the FSMC_Bank4 registers to their reset values */
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0xa0
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as *mut FSMC_Bank4_TypeDef)).PCR4
                                    as *mut uint32_t,
                                0x18 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0xa0
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as *mut FSMC_Bank4_TypeDef)).SR4
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0xa0
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as *mut FSMC_Bank4_TypeDef)).PMEM4
                                    as *mut uint32_t,
                                0xfcfcfcfc as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0xa0
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as *mut FSMC_Bank4_TypeDef)).PATT4
                                    as *mut uint32_t,
                                0xfcfcfcfc as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0xa0
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as *mut FSMC_Bank4_TypeDef)).PIO4
                                    as *mut uint32_t,
                                0xfcfcfcfc as libc::c_uint);
}
/* *
  * @brief  Initializes the FSMC NOR/SRAM Banks according to the specified
  *         parameters in the FSMC_NORSRAMInitStruct.
  * @param  FSMC_NORSRAMInitStruct : pointer to a FSMC_NORSRAMInitTypeDef
  *         structure that contains the configuration information for 
  *        the FSMC NOR/SRAM specified Banks.                       
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_NORSRAMInit(mut FSMC_NORSRAMInitStruct:
                                              *mut FSMC_NORSRAMInitTypeDef) {
    /* Check the parameters */
    /* Bank1 NOR/SRAM control register configuration */
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as
                                            *mut FSMC_Bank1_TypeDef)).BTCR[(*FSMC_NORSRAMInitStruct).FSMC_Bank
                                                                               as
                                                                               usize]
                                    as *mut uint32_t,
                                (*FSMC_NORSRAMInitStruct).FSMC_DataAddressMux
                                    |
                                    (*FSMC_NORSRAMInitStruct).FSMC_MemoryType
                                    |
                                    (*FSMC_NORSRAMInitStruct).FSMC_MemoryDataWidth
                                    |
                                    (*FSMC_NORSRAMInitStruct).FSMC_BurstAccessMode
                                    |
                                    (*FSMC_NORSRAMInitStruct).FSMC_AsynchronousWait
                                    |
                                    (*FSMC_NORSRAMInitStruct).FSMC_WaitSignalPolarity
                                    | (*FSMC_NORSRAMInitStruct).FSMC_WrapMode
                                    |
                                    (*FSMC_NORSRAMInitStruct).FSMC_WaitSignalActive
                                    |
                                    (*FSMC_NORSRAMInitStruct).FSMC_WriteOperation
                                    |
                                    (*FSMC_NORSRAMInitStruct).FSMC_WaitSignal
                                    |
                                    (*FSMC_NORSRAMInitStruct).FSMC_ExtendedMode
                                    |
                                    (*FSMC_NORSRAMInitStruct).FSMC_WriteBurst);
    if (*FSMC_NORSRAMInitStruct).FSMC_MemoryType ==
           0x8 as libc::c_int as uint32_t {
        let ref mut fresh0 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank1_TypeDef)).BTCR[(*FSMC_NORSRAMInitStruct).FSMC_Bank
                                                      as usize];
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x40 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    }
    /* Bank1 NOR/SRAM timing register configuration */
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as
                                            *mut FSMC_Bank1_TypeDef)).BTCR[(*FSMC_NORSRAMInitStruct).FSMC_Bank.wrapping_add(1
                                                                                                                                as
                                                                                                                                libc::c_int
                                                                                                                                as
                                                                                                                                libc::c_uint)
                                                                               as
                                                                               usize]
                                    as *mut uint32_t,
                                (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_AddressSetupTime
                                    |
                                    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_AddressHoldTime
                                        << 4 as libc::c_int |
                                    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_DataSetupTime
                                        << 8 as libc::c_int |
                                    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_BusTurnAroundDuration
                                        << 16 as libc::c_int |
                                    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_CLKDivision
                                        << 20 as libc::c_int |
                                    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_DataLatency
                                        << 24 as libc::c_int |
                                    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_AccessMode);
    /* Bank1 NOR/SRAM timing register for write configuration, if extended mode is used */
    if (*FSMC_NORSRAMInitStruct).FSMC_ExtendedMode ==
           0x4000 as libc::c_int as uint32_t {
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x104
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank1E_TypeDef)).BWTR[(*FSMC_NORSRAMInitStruct).FSMC_Bank
                                                                                    as
                                                                                    usize]
                                        as *mut uint32_t,
                                    (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_AddressSetupTime
                                        |
                                        (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_AddressHoldTime
                                            << 4 as libc::c_int |
                                        (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_DataSetupTime
                                            << 8 as libc::c_int |
                                        (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_CLKDivision
                                            << 20 as libc::c_int |
                                        (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_DataLatency
                                            << 24 as libc::c_int |
                                        (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_AccessMode)
    } else {
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x104
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank1E_TypeDef)).BWTR[(*FSMC_NORSRAMInitStruct).FSMC_Bank
                                                                                    as
                                                                                    usize]
                                        as *mut uint32_t,
                                    0xfffffff as libc::c_int as uint32_t)
    };
}
/* *
  * @brief  Initializes the FSMC NAND Banks according to the specified 
  *         parameters in the FSMC_NANDInitStruct.
  * @param  FSMC_NANDInitStruct : pointer to a FSMC_NANDInitTypeDef 
  *         structure that contains the configuration information for the FSMC 
  *         NAND specified Banks.                       
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_NANDInit(mut FSMC_NANDInitStruct:
                                           *mut FSMC_NANDInitTypeDef) {
    let mut tmppcr: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmppmem: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmppatt: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Set the tmppcr value according to FSMC_NANDInitStruct parameters */
    tmppcr =
        (*FSMC_NANDInitStruct).FSMC_Waitfeature |
            0x8 as libc::c_int as uint32_t |
            (*FSMC_NANDInitStruct).FSMC_MemoryDataWidth |
            (*FSMC_NANDInitStruct).FSMC_ECC |
            (*FSMC_NANDInitStruct).FSMC_ECCPageSize |
            (*FSMC_NANDInitStruct).FSMC_TCLRSetupTime << 9 as libc::c_int |
            (*FSMC_NANDInitStruct).FSMC_TARSetupTime << 13 as libc::c_int;
    /* Set tmppmem value according to FSMC_CommonSpaceTimingStructure parameters */
    tmppmem =
        (*(*FSMC_NANDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_SetupTime
            |
            (*(*FSMC_NANDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_WaitSetupTime
                << 8 as libc::c_int |
            (*(*FSMC_NANDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_HoldSetupTime
                << 16 as libc::c_int |
            (*(*FSMC_NANDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_HiZSetupTime
                << 24 as libc::c_int;
    /* Set tmppatt value according to FSMC_AttributeSpaceTimingStructure parameters */
    tmppatt =
        (*(*FSMC_NANDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_SetupTime
            |
            (*(*FSMC_NANDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_WaitSetupTime
                << 8 as libc::c_int |
            (*(*FSMC_NANDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_HoldSetupTime
                << 16 as libc::c_int |
            (*(*FSMC_NANDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_HiZSetupTime
                << 24 as libc::c_int;
    if (*FSMC_NANDInitStruct).FSMC_Bank == 0x10 as libc::c_int as uint32_t {
        /* FSMC_Bank2_NAND registers configuration */
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x60
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank2_TypeDef)).PCR2
                                        as *mut uint32_t, tmppcr);
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x60
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank2_TypeDef)).PMEM2
                                        as *mut uint32_t, tmppmem);
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x60
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank2_TypeDef)).PATT2
                                        as *mut uint32_t, tmppatt)
    } else {
        /* FSMC_Bank3_NAND registers configuration */
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x80
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank3_TypeDef)).PCR3
                                        as *mut uint32_t, tmppcr);
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x80
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank3_TypeDef)).PMEM3
                                        as *mut uint32_t, tmppmem);
        ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                                 libc::c_uint).wrapping_add(0x80
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                as
                                                *mut FSMC_Bank3_TypeDef)).PATT3
                                        as *mut uint32_t, tmppatt)
    };
}
/* *
  * @brief  Initializes the FSMC PCCARD Bank according to the specified 
  *         parameters in the FSMC_PCCARDInitStruct.
  * @param  FSMC_PCCARDInitStruct : pointer to a FSMC_PCCARDInitTypeDef
  *         structure that contains the configuration information for the FSMC 
  *         PCCARD Bank.                       
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_PCCARDInit(mut FSMC_PCCARDInitStruct:
                                             *mut FSMC_PCCARDInitTypeDef) {
    /* Check the parameters */
    /* Set the PCR4 register value according to FSMC_PCCARDInitStruct parameters */
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0xa0
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as *mut FSMC_Bank4_TypeDef)).PCR4
                                    as *mut uint32_t,
                                (*FSMC_PCCARDInitStruct).FSMC_Waitfeature |
                                    0x10 as libc::c_int as uint32_t |
                                    (*FSMC_PCCARDInitStruct).FSMC_TCLRSetupTime
                                        << 9 as libc::c_int |
                                    (*FSMC_PCCARDInitStruct).FSMC_TARSetupTime
                                        << 13 as libc::c_int);
    /* Set PMEM4 register value according to FSMC_CommonSpaceTimingStructure parameters */
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0xa0
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as *mut FSMC_Bank4_TypeDef)).PMEM4
                                    as *mut uint32_t,
                                (*(*FSMC_PCCARDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_SetupTime
                                    |
                                    (*(*FSMC_PCCARDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_WaitSetupTime
                                        << 8 as libc::c_int |
                                    (*(*FSMC_PCCARDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_HoldSetupTime
                                        << 16 as libc::c_int |
                                    (*(*FSMC_PCCARDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_HiZSetupTime
                                        << 24 as libc::c_int);
    /* Set PATT4 register value according to FSMC_AttributeSpaceTimingStructure parameters */
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0xa0
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as *mut FSMC_Bank4_TypeDef)).PATT4
                                    as *mut uint32_t,
                                (*(*FSMC_PCCARDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_SetupTime
                                    |
                                    (*(*FSMC_PCCARDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_WaitSetupTime
                                        << 8 as libc::c_int |
                                    (*(*FSMC_PCCARDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_HoldSetupTime
                                        << 16 as libc::c_int |
                                    (*(*FSMC_PCCARDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_HiZSetupTime
                                        << 24 as libc::c_int);
    /* Set PIO4 register value according to FSMC_IOSpaceTimingStructure parameters */
    ::core::ptr::write_volatile(&mut (*((0xa0000000 as
                                             libc::c_uint).wrapping_add(0xa0
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                            as *mut FSMC_Bank4_TypeDef)).PIO4
                                    as *mut uint32_t,
                                (*(*FSMC_PCCARDInitStruct).FSMC_IOSpaceTimingStruct).FSMC_SetupTime
                                    |
                                    (*(*FSMC_PCCARDInitStruct).FSMC_IOSpaceTimingStruct).FSMC_WaitSetupTime
                                        << 8 as libc::c_int |
                                    (*(*FSMC_PCCARDInitStruct).FSMC_IOSpaceTimingStruct).FSMC_HoldSetupTime
                                        << 16 as libc::c_int |
                                    (*(*FSMC_PCCARDInitStruct).FSMC_IOSpaceTimingStruct).FSMC_HiZSetupTime
                                        << 24 as libc::c_int);
}
/* *
  * @brief  Fills each FSMC_NORSRAMInitStruct member with its default value.
  * @param  FSMC_NORSRAMInitStruct: pointer to a FSMC_NORSRAMInitTypeDef 
  *         structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_NORSRAMStructInit(mut FSMC_NORSRAMInitStruct:
                                                    *mut FSMC_NORSRAMInitTypeDef) {
    /* Reset NOR/SRAM Init structure parameters values */
    (*FSMC_NORSRAMInitStruct).FSMC_Bank = 0 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_DataAddressMux =
        0x2 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_MemoryType = 0 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_MemoryDataWidth =
        0 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_BurstAccessMode =
        0 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_AsynchronousWait =
        0 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_WaitSignalPolarity =
        0 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_WrapMode = 0 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_WaitSignalActive =
        0 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_WriteOperation =
        0x1000 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_WaitSignal =
        0x2000 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_ExtendedMode =
        0 as libc::c_int as uint32_t;
    (*FSMC_NORSRAMInitStruct).FSMC_WriteBurst = 0 as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_AddressSetupTime
        = 0xf as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_AddressHoldTime
        = 0xf as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_DataSetupTime
        = 0xff as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_BusTurnAroundDuration
        = 0xf as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_CLKDivision =
        0xf as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_DataLatency =
        0xf as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_ReadWriteTimingStruct).FSMC_AccessMode =
        0 as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_AddressSetupTime
        = 0xf as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_AddressHoldTime =
        0xf as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_DataSetupTime =
        0xff as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_BusTurnAroundDuration
        = 0xf as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_CLKDivision =
        0xf as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_DataLatency =
        0xf as libc::c_int as uint32_t;
    (*(*FSMC_NORSRAMInitStruct).FSMC_WriteTimingStruct).FSMC_AccessMode =
        0 as libc::c_int as uint32_t;
}
/* *
  * @brief  Fills each FSMC_NANDInitStruct member with its default value.
  * @param  FSMC_NANDInitStruct: pointer to a FSMC_NANDInitTypeDef 
  *         structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_NANDStructInit(mut FSMC_NANDInitStruct:
                                                 *mut FSMC_NANDInitTypeDef) {
    /* Reset NAND Init structure parameters values */
    (*FSMC_NANDInitStruct).FSMC_Bank = 0x10 as libc::c_int as uint32_t;
    (*FSMC_NANDInitStruct).FSMC_Waitfeature = 0 as libc::c_int as uint32_t;
    (*FSMC_NANDInitStruct).FSMC_MemoryDataWidth =
        0 as libc::c_int as uint32_t;
    (*FSMC_NANDInitStruct).FSMC_ECC = 0 as libc::c_int as uint32_t;
    (*FSMC_NANDInitStruct).FSMC_ECCPageSize = 0 as libc::c_int as uint32_t;
    (*FSMC_NANDInitStruct).FSMC_TCLRSetupTime = 0 as libc::c_int as uint32_t;
    (*FSMC_NANDInitStruct).FSMC_TARSetupTime = 0 as libc::c_int as uint32_t;
    (*(*FSMC_NANDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_SetupTime =
        0xfc as libc::c_int as uint32_t;
    (*(*FSMC_NANDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_WaitSetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_NANDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_HoldSetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_NANDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_HiZSetupTime =
        0xfc as libc::c_int as uint32_t;
    (*(*FSMC_NANDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_SetupTime =
        0xfc as libc::c_int as uint32_t;
    (*(*FSMC_NANDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_WaitSetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_NANDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_HoldSetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_NANDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_HiZSetupTime
        = 0xfc as libc::c_int as uint32_t;
}
/* *
  * @brief  Fills each FSMC_PCCARDInitStruct member with its default value.
  * @param  FSMC_PCCARDInitStruct: pointer to a FSMC_PCCARDInitTypeDef 
  *         structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_PCCARDStructInit(mut FSMC_PCCARDInitStruct:
                                                   *mut FSMC_PCCARDInitTypeDef) {
    /* Reset PCCARD Init structure parameters values */
    (*FSMC_PCCARDInitStruct).FSMC_Waitfeature = 0 as libc::c_int as uint32_t;
    (*FSMC_PCCARDInitStruct).FSMC_TCLRSetupTime =
        0 as libc::c_int as uint32_t;
    (*FSMC_PCCARDInitStruct).FSMC_TARSetupTime = 0 as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_SetupTime =
        0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_WaitSetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_HoldSetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_CommonSpaceTimingStruct).FSMC_HiZSetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_SetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_WaitSetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_HoldSetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_AttributeSpaceTimingStruct).FSMC_HiZSetupTime
        = 0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_IOSpaceTimingStruct).FSMC_SetupTime =
        0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_IOSpaceTimingStruct).FSMC_WaitSetupTime =
        0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_IOSpaceTimingStruct).FSMC_HoldSetupTime =
        0xfc as libc::c_int as uint32_t;
    (*(*FSMC_PCCARDInitStruct).FSMC_IOSpaceTimingStruct).FSMC_HiZSetupTime =
        0xfc as libc::c_int as uint32_t;
}
/* *
  * @brief  Enables or disables the specified NOR/SRAM Memory Bank.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank1_NORSRAM1: FSMC Bank1 NOR/SRAM1  
  *     @arg FSMC_Bank1_NORSRAM2: FSMC Bank1 NOR/SRAM2 
  *     @arg FSMC_Bank1_NORSRAM3: FSMC Bank1 NOR/SRAM3 
  *     @arg FSMC_Bank1_NORSRAM4: FSMC Bank1 NOR/SRAM4 
  * @param  NewState: new state of the FSMC_Bank. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_NORSRAMCmd(mut FSMC_Bank: uint32_t,
                                         mut NewState: FunctionalState) {
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected NOR/SRAM Bank by setting the PBKEN bit in the BCRx register */
        let ref mut fresh1 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank1_TypeDef)).BTCR[FSMC_Bank as usize];
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected NOR/SRAM Bank by clearing the PBKEN bit in the BCRx register */
        let ref mut fresh2 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank1_TypeDef)).BTCR[FSMC_Bank as usize];
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xffffe as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the specified NAND Memory Bank.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank2_NAND: FSMC Bank2 NAND 
  *     @arg FSMC_Bank3_NAND: FSMC Bank3 NAND
  * @param  NewState: new state of the FSMC_Bank. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_NANDCmd(mut FSMC_Bank: uint32_t,
                                      mut NewState: FunctionalState) {
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected NAND Bank by setting the PBKEN bit in the PCRx register */
        if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
            let ref mut fresh3 =
                (*((0xa0000000 as
                        libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                       libc::c_uint) as
                       *mut FSMC_Bank2_TypeDef)).PCR2;
            ::core::ptr::write_volatile(fresh3,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x4 as libc::c_int as uint32_t)
                                            as uint32_t as uint32_t)
        } else {
            let ref mut fresh4 =
                (*((0xa0000000 as
                        libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                       libc::c_uint) as
                       *mut FSMC_Bank3_TypeDef)).PCR3;
            ::core::ptr::write_volatile(fresh4,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x4 as libc::c_int as uint32_t)
                                            as uint32_t as uint32_t)
        }
    } else if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
        let ref mut fresh5 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank2_TypeDef)).PCR2;
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xffffb as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        let ref mut fresh6 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank3_TypeDef)).PCR3;
        ::core::ptr::write_volatile(fresh6,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xffffb as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    };
}
/* Disable the selected NAND Bank by clearing the PBKEN bit in the PCRx register */
/* *
  * @brief  Enables or disables the PCCARD Memory Bank.
  * @param  NewState: new state of the PCCARD Memory Bank.  
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_PCCARDCmd(mut NewState: FunctionalState) {
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the PCCARD Bank by setting the PBKEN bit in the PCR4 register */
        let ref mut fresh7 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0xa0 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank4_TypeDef)).PCR4;
        ::core::ptr::write_volatile(fresh7,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the PCCARD Bank by clearing the PBKEN bit in the PCR4 register */
        let ref mut fresh8 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0xa0 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank4_TypeDef)).PCR4;
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xffffb as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the FSMC NAND ECC feature.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank2_NAND: FSMC Bank2 NAND 
  *     @arg FSMC_Bank3_NAND: FSMC Bank3 NAND
  * @param  NewState: new state of the FSMC NAND ECC feature.  
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_NANDECCCmd(mut FSMC_Bank: uint32_t,
                                         mut NewState: FunctionalState) {
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected NAND Bank ECC function by setting the ECCEN bit in the PCRx register */
        if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
            let ref mut fresh9 =
                (*((0xa0000000 as
                        libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                       libc::c_uint) as
                       *mut FSMC_Bank2_TypeDef)).PCR2;
            ::core::ptr::write_volatile(fresh9,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x40 as libc::c_int as uint32_t)
                                            as uint32_t as uint32_t)
        } else {
            let ref mut fresh10 =
                (*((0xa0000000 as
                        libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                       libc::c_uint) as
                       *mut FSMC_Bank3_TypeDef)).PCR3;
            ::core::ptr::write_volatile(fresh10,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x40 as libc::c_int as uint32_t)
                                            as uint32_t as uint32_t)
        }
    } else if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
        let ref mut fresh11 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank2_TypeDef)).PCR2;
        ::core::ptr::write_volatile(fresh11,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xfffbf as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        let ref mut fresh12 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank3_TypeDef)).PCR3;
        ::core::ptr::write_volatile(fresh12,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xfffbf as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    };
}
/* Disable the selected NAND Bank ECC function by clearing the ECCEN bit in the PCRx register */
/* *
  * @brief  Returns the error correction code register value.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank2_NAND: FSMC Bank2 NAND 
  *     @arg FSMC_Bank3_NAND: FSMC Bank3 NAND
  * @retval The Error Correction Code (ECC) value.
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_GetECC(mut FSMC_Bank: uint32_t) -> uint32_t {
    let mut eccval: uint32_t = 0 as libc::c_int as uint32_t;
    if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
        /* Get the ECCR2 register value */
        eccval =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank2_TypeDef)).ECCR2
    } else {
        /* Get the ECCR3 register value */
        eccval =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank3_TypeDef)).ECCR3
    }
    /* Return the error correction code value */
    return eccval;
}
/* *
  * @brief  Enables or disables the specified FSMC interrupts.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank2_NAND: FSMC Bank2 NAND 
  *     @arg FSMC_Bank3_NAND: FSMC Bank3 NAND
  *     @arg FSMC_Bank4_PCCARD: FSMC Bank4 PCCARD
  * @param  FSMC_IT: specifies the FSMC interrupt sources to be enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg FSMC_IT_RisingEdge: Rising edge detection interrupt. 
  *     @arg FSMC_IT_Level: Level edge detection interrupt.
  *     @arg FSMC_IT_FallingEdge: Falling edge detection interrupt.
  * @param  NewState: new state of the specified FSMC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_ITConfig(mut FSMC_Bank: uint32_t,
                                       mut FSMC_IT: uint32_t,
                                       mut NewState: FunctionalState) {
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected FSMC_Bank2 interrupts */
        if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
            let ref mut fresh13 =
                (*((0xa0000000 as
                        libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                       libc::c_uint) as
                       *mut FSMC_Bank2_TypeDef)).SR2;
            ::core::ptr::write_volatile(fresh13,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint | FSMC_IT) as
                                            uint32_t as uint32_t)
        } else if FSMC_Bank == 0x100 as libc::c_int as uint32_t {
            let ref mut fresh14 =
                (*((0xa0000000 as
                        libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                       libc::c_uint) as
                       *mut FSMC_Bank3_TypeDef)).SR3;
            ::core::ptr::write_volatile(fresh14,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint | FSMC_IT) as
                                            uint32_t as uint32_t)
        } else {
            /* Enable the selected FSMC_Bank3 interrupts */
            /* Enable the selected FSMC_Bank4 interrupts */
            let ref mut fresh15 =
                (*((0xa0000000 as
                        libc::c_uint).wrapping_add(0xa0 as libc::c_int as
                                                       libc::c_uint) as
                       *mut FSMC_Bank4_TypeDef)).SR4;
            ::core::ptr::write_volatile(fresh15,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint | FSMC_IT) as
                                            uint32_t as uint32_t)
        }
    } else if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
        let ref mut fresh16 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank2_TypeDef)).SR2;
        ::core::ptr::write_volatile(fresh16,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !FSMC_IT) as
                                        uint32_t as uint32_t)
    } else if FSMC_Bank == 0x100 as libc::c_int as uint32_t {
        let ref mut fresh17 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank3_TypeDef)).SR3;
        ::core::ptr::write_volatile(fresh17,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !FSMC_IT) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected FSMC_Bank2 interrupts */
        /* Disable the selected FSMC_Bank3 interrupts */
        /* Disable the selected FSMC_Bank4 interrupts */
        let ref mut fresh18 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0xa0 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank4_TypeDef)).SR4;
        ::core::ptr::write_volatile(fresh18,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !FSMC_IT) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Checks whether the specified FSMC flag is set or not.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank2_NAND: FSMC Bank2 NAND 
  *     @arg FSMC_Bank3_NAND: FSMC Bank3 NAND
  *     @arg FSMC_Bank4_PCCARD: FSMC Bank4 PCCARD
  * @param  FSMC_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg FSMC_FLAG_RisingEdge: Rising egde detection Flag.
  *     @arg FSMC_FLAG_Level: Level detection Flag.
  *     @arg FSMC_FLAG_FallingEdge: Falling egde detection Flag.
  *     @arg FSMC_FLAG_FEMPT: Fifo empty Flag. 
  * @retval The new state of FSMC_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_GetFlagStatus(mut FSMC_Bank: uint32_t,
                                            mut FSMC_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    let mut tmpsr: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
        tmpsr =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank2_TypeDef)).SR2
    } else if FSMC_Bank == 0x100 as libc::c_int as uint32_t {
        tmpsr =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank3_TypeDef)).SR3
    } else {
        /* FSMC_Bank4_PCCARD*/
        tmpsr =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0xa0 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank4_TypeDef)).SR4
    }
    /* Get the flag status */
    if tmpsr & FSMC_FLAG != RESET as libc::c_int as uint16_t as libc::c_uint {
        bitstatus = SET
    } else { bitstatus = RESET }
    /* Return the flag status */
    return bitstatus;
}
/* *
  * @brief  Clears the FSMC's pending flags.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank2_NAND: FSMC Bank2 NAND 
  *     @arg FSMC_Bank3_NAND: FSMC Bank3 NAND
  *     @arg FSMC_Bank4_PCCARD: FSMC Bank4 PCCARD
  * @param  FSMC_FLAG: specifies the flag to clear.
  *   This parameter can be any combination of the following values:
  *     @arg FSMC_FLAG_RisingEdge: Rising egde detection Flag.
  *     @arg FSMC_FLAG_Level: Level detection Flag.
  *     @arg FSMC_FLAG_FallingEdge: Falling egde detection Flag.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_ClearFlag(mut FSMC_Bank: uint32_t,
                                        mut FSMC_FLAG: uint32_t) {
    /* Check the parameters */
    if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
        let ref mut fresh19 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank2_TypeDef)).SR2;
        ::core::ptr::write_volatile(fresh19,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !FSMC_FLAG) as
                                        uint32_t as uint32_t)
    } else if FSMC_Bank == 0x100 as libc::c_int as uint32_t {
        let ref mut fresh20 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank3_TypeDef)).SR3;
        ::core::ptr::write_volatile(fresh20,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !FSMC_FLAG) as
                                        uint32_t as uint32_t)
    } else {
        /* FSMC_Bank4_PCCARD*/
        let ref mut fresh21 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0xa0 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank4_TypeDef)).SR4;
        ::core::ptr::write_volatile(fresh21,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !FSMC_FLAG) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Checks whether the specified FSMC interrupt has occurred or not.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank2_NAND: FSMC Bank2 NAND 
  *     @arg FSMC_Bank3_NAND: FSMC Bank3 NAND
  *     @arg FSMC_Bank4_PCCARD: FSMC Bank4 PCCARD
  * @param  FSMC_IT: specifies the FSMC interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg FSMC_IT_RisingEdge: Rising edge detection interrupt. 
  *     @arg FSMC_IT_Level: Level edge detection interrupt.
  *     @arg FSMC_IT_FallingEdge: Falling edge detection interrupt. 
  * @retval The new state of FSMC_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_GetITStatus(mut FSMC_Bank: uint32_t,
                                          mut FSMC_IT: uint32_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut tmpsr: uint32_t = 0 as libc::c_int as uint32_t;
    let mut itstatus: uint32_t = 0 as libc::c_int as uint32_t;
    let mut itenable: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
        tmpsr =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank2_TypeDef)).SR2
    } else if FSMC_Bank == 0x100 as libc::c_int as uint32_t {
        tmpsr =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank3_TypeDef)).SR3
    } else {
        /* FSMC_Bank4_PCCARD*/
        tmpsr =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0xa0 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank4_TypeDef)).SR4
    }
    itstatus = tmpsr & FSMC_IT;
    itenable = tmpsr & FSMC_IT >> 3 as libc::c_int;
    if itstatus != RESET as libc::c_int as uint32_t &&
           itenable != RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_fsmc.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the FSMC firmware 
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
/* * @addtogroup FSMC
  * @{
  */
/* * @defgroup FSMC_Exported_Types
  * @{
  */
/* * 
  * @brief  Timing parameters For NOR/SRAM Banks  
  */
/* !< Defines the number of HCLK cycles to configure
                                             the duration of the address setup time. 
                                             This parameter can be a value between 0 and 0xF.
                                             @note: It is not used with synchronous NOR Flash memories. */
/* !< Defines the number of HCLK cycles to configure
                                             the duration of the address hold time.
                                             This parameter can be a value between 0 and 0xF. 
                                             @note: It is not used with synchronous NOR Flash memories.*/
/* !< Defines the number of HCLK cycles to configure
                                             the duration of the data setup time.
                                             This parameter can be a value between 0 and 0xFF.
                                             @note: It is used for SRAMs, ROMs and asynchronous multiplexed NOR Flash memories. */
/* !< Defines the number of HCLK cycles to configure
                                             the duration of the bus turnaround.
                                             This parameter can be a value between 0 and 0xF.
                                             @note: It is only used for multiplexed NOR Flash memories. */
/* !< Defines the period of CLK clock output signal, expressed in number of HCLK cycles.
                                             This parameter can be a value between 1 and 0xF.
                                             @note: This parameter is not used for asynchronous NOR Flash, SRAM or ROM accesses. */
/* !< Defines the number of memory clock cycles to issue
                                             to the memory before getting the first data.
                                             The value of this parameter depends on the memory type as shown below:
                                              - It must be set to 0 in case of a CRAM
                                              - It is don't care in asynchronous NOR, SRAM or ROM accesses
                                              - It may assume a value between 0 and 0xF in NOR Flash memories
                                                with synchronous burst mode enable */
/* !< Specifies the asynchronous access mode. 
                                             This parameter can be a value of @ref FSMC_Access_Mode */
/* * 
  * @brief  FSMC NOR/SRAM Init structure definition
  */
/* !< Specifies the NOR/SRAM memory bank that will be used.
                                          This parameter can be a value of @ref FSMC_NORSRAM_Bank */
/* !< Specifies whether the address and data values are
                                          multiplexed on the databus or not. 
                                          This parameter can be a value of @ref FSMC_Data_Address_Bus_Multiplexing */
/* !< Specifies the type of external memory attached to
                                          the corresponding memory bank.
                                          This parameter can be a value of @ref FSMC_Memory_Type */
/* !< Specifies the external memory device width.
                                          This parameter can be a value of @ref FSMC_Data_Width */
/* !< Enables or disables the burst access mode for Flash memory,
                                          valid only with synchronous burst Flash memories.
                                          This parameter can be a value of @ref FSMC_Burst_Access_Mode */
/* !< Enables or disables wait signal during asynchronous transfers,
                                          valid only with asynchronous Flash memories.
                                          This parameter can be a value of @ref FSMC_AsynchronousWait */
/* !< Specifies the wait signal polarity, valid only when accessing
                                          the Flash memory in burst mode.
                                          This parameter can be a value of @ref FSMC_Wait_Signal_Polarity */
/* !< Enables or disables the Wrapped burst access mode for Flash
                                          memory, valid only when accessing Flash memories in burst mode.
                                          This parameter can be a value of @ref FSMC_Wrap_Mode */
/* !< Specifies if the wait signal is asserted by the memory one
                                          clock cycle before the wait state or during the wait state,
                                          valid only when accessing memories in burst mode. 
                                          This parameter can be a value of @ref FSMC_Wait_Timing */
/* !< Enables or disables the write operation in the selected bank by the FSMC. 
                                          This parameter can be a value of @ref FSMC_Write_Operation */
/* !< Enables or disables the wait-state insertion via wait
                                          signal, valid for Flash memory access in burst mode. 
                                          This parameter can be a value of @ref FSMC_Wait_Signal */
/* !< Enables or disables the extended mode.
                                          This parameter can be a value of @ref FSMC_Extended_Mode */
/* !< Enables or disables the write burst operation.
                                          This parameter can be a value of @ref FSMC_Write_Burst */
/* !< Timing Parameters for write and read access if the  ExtendedMode is not used*/
/* !< Timing Parameters for write access if the  ExtendedMode is used*/
/* * 
  * @brief  Timing parameters For FSMC NAND and PCCARD Banks
  */
/* !< Defines the number of HCLK cycles to setup address before
                                     the command assertion for NAND-Flash read or write access
                                     to common/Attribute or I/O memory space (depending on
                                     the memory space timing to be configured).
                                     This parameter can be a value between 0 and 0xFF.*/
/* !< Defines the minimum number of HCLK cycles to assert the
                                     command for NAND-Flash read or write access to
                                     common/Attribute or I/O memory space (depending on the
                                     memory space timing to be configured). 
                                     This parameter can be a number between 0x00 and 0xFF */
/* !< Defines the number of HCLK clock cycles to hold address
                                     (and data for write access) after the command deassertion
                                     for NAND-Flash read or write access to common/Attribute
                                     or I/O memory space (depending on the memory space timing
                                     to be configured).
                                     This parameter can be a number between 0x00 and 0xFF */
/* !< Defines the number of HCLK clock cycles during which the
                                     databus is kept in HiZ after the start of a NAND-Flash
                                     write access to common/Attribute or I/O memory space (depending
                                     on the memory space timing to be configured).
                                     This parameter can be a number between 0x00 and 0xFF */
/* * 
  * @brief  FSMC NAND Init structure definition
  */
/* !< Specifies the NAND memory bank that will be used.
                                      This parameter can be a value of @ref FSMC_NAND_Bank */
/* !< Enables or disables the Wait feature for the NAND Memory Bank.
                                       This parameter can be any value of @ref FSMC_Wait_feature */
/* !< Specifies the external memory device width.
                                       This parameter can be any value of @ref FSMC_Data_Width */
/* !< Enables or disables the ECC computation.
                                       This parameter can be any value of @ref FSMC_ECC */
/* !< Defines the page size for the extended ECC.
                                       This parameter can be any value of @ref FSMC_ECC_Page_Size */
/* !< Defines the number of HCLK cycles to configure the
                                       delay between CLE low and RE low.
                                       This parameter can be a value between 0 and 0xFF. */
/* !< Defines the number of HCLK cycles to configure the
                                       delay between ALE low and RE low.
                                       This parameter can be a number between 0x0 and 0xFF */
/* !< FSMC Common Space Timing */
/* !< FSMC Attribute Space Timing */
/* * 
  * @brief  FSMC PCCARD Init structure definition
  */
/* !< Enables or disables the Wait feature for the Memory Bank.
                                    This parameter can be any value of @ref FSMC_Wait_feature */
/* !< Defines the number of HCLK cycles to configure the
                                     delay between CLE low and RE low.
                                     This parameter can be a value between 0 and 0xFF. */
/* !< Defines the number of HCLK cycles to configure the
                                     delay between ALE low and RE low.
                                     This parameter can be a number between 0x0 and 0xFF */
/* !< FSMC Common Space Timing */
/* !< FSMC Attribute Space Timing */
/* !< FSMC IO Space Timing */
/* *
  * @}
  */
/* * @defgroup FSMC_Exported_Constants
  * @{
  */
/* * @defgroup FSMC_NORSRAM_Bank 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_NAND_Bank 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_PCCARD_Bank 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup NOR_SRAM_Controller 
  * @{
  */
/* * @defgroup FSMC_Data_Address_Bus_Multiplexing 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Memory_Type 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Data_Width 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Burst_Access_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_AsynchronousWait 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Wait_Signal_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Wrap_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Wait_Timing 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Write_Operation 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Wait_Signal 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Extended_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Write_Burst 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Address_Setup_Time 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Address_Hold_Time 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Data_Setup_Time 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Bus_Turn_around_Duration 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_CLK_Division 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Data_Latency 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Access_Mode 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup NAND_PCCARD_Controller 
  * @{
  */
/* * @defgroup FSMC_Wait_feature 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_ECC 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_ECC_Page_Size 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_TCLR_Setup_Time 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_TAR_Setup_Time 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Setup_Time 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Wait_Setup_Time 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Hold_Setup_Time 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_HiZ_Setup_Time 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Interrupt_sources 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Flags 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FSMC_Exported_Functions
  * @{
  */
/* *
  * @brief  Clears the FSMC's interrupt pending bits.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank2_NAND: FSMC Bank2 NAND 
  *     @arg FSMC_Bank3_NAND: FSMC Bank3 NAND
  *     @arg FSMC_Bank4_PCCARD: FSMC Bank4 PCCARD
  * @param  FSMC_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg FSMC_IT_RisingEdge: Rising edge detection interrupt. 
  *     @arg FSMC_IT_Level: Level edge detection interrupt.
  *     @arg FSMC_IT_FallingEdge: Falling edge detection interrupt.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FSMC_ClearITPendingBit(mut FSMC_Bank: uint32_t,
                                                mut FSMC_IT: uint32_t) {
    /* Check the parameters */
    if FSMC_Bank == 0x10 as libc::c_int as uint32_t {
        let ref mut fresh22 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x60 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank2_TypeDef)).SR2;
        ::core::ptr::write_volatile(fresh22,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(FSMC_IT >> 3 as libc::c_int)) as
                                        uint32_t as uint32_t)
    } else if FSMC_Bank == 0x100 as libc::c_int as uint32_t {
        let ref mut fresh23 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0x80 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank3_TypeDef)).SR3;
        ::core::ptr::write_volatile(fresh23,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(FSMC_IT >> 3 as libc::c_int)) as
                                        uint32_t as uint32_t)
    } else {
        /* FSMC_Bank4_PCCARD*/
        let ref mut fresh24 =
            (*((0xa0000000 as
                    libc::c_uint).wrapping_add(0xa0 as libc::c_int as
                                                   libc::c_uint) as
                   *mut FSMC_Bank4_TypeDef)).SR4;
        ::core::ptr::write_volatile(fresh24,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh24
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(FSMC_IT >> 3 as libc::c_int)) as
                                        uint32_t as uint32_t)
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
