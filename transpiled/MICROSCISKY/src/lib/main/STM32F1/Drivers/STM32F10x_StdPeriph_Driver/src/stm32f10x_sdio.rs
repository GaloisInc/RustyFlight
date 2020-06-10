use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
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
  * @brief SD host Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SDIO_TypeDef {
    pub POWER: uint32_t,
    pub CLKCR: uint32_t,
    pub ARG: uint32_t,
    pub CMD: uint32_t,
    pub RESPCMD: uint32_t,
    pub RESP1: uint32_t,
    pub RESP2: uint32_t,
    pub RESP3: uint32_t,
    pub RESP4: uint32_t,
    pub DTIMER: uint32_t,
    pub DLEN: uint32_t,
    pub DCTRL: uint32_t,
    pub DCOUNT: uint32_t,
    pub STA: uint32_t,
    pub ICR: uint32_t,
    pub MASK: uint32_t,
    pub RESERVED0: [uint32_t; 2],
    pub FIFOCNT: uint32_t,
    pub RESERVED1: [uint32_t; 13],
    pub FIFO: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SDIO_InitTypeDef {
    pub SDIO_ClockEdge: uint32_t,
    pub SDIO_ClockBypass: uint32_t,
    pub SDIO_ClockPowerSave: uint32_t,
    pub SDIO_BusWide: uint32_t,
    pub SDIO_HardwareFlowControl: uint32_t,
    pub SDIO_ClockDiv: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SDIO_CmdInitTypeDef {
    pub SDIO_Argument: uint32_t,
    pub SDIO_CmdIndex: uint32_t,
    pub SDIO_Response: uint32_t,
    pub SDIO_Wait: uint32_t,
    pub SDIO_CPSM: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SDIO_DataInitTypeDef {
    pub SDIO_DataTimeOut: uint32_t,
    pub SDIO_DataLength: uint32_t,
    pub SDIO_DataBlockSize: uint32_t,
    pub SDIO_TransferDir: uint32_t,
    pub SDIO_TransferMode: uint32_t,
    pub SDIO_DPSM: uint32_t,
}
/* *
  * @}
  */
/* * @defgroup SDIO_Private_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the SDIO peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_DeInit() {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).POWER as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).CLKCR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).ARG as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).CMD as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).DTIMER as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).DLEN as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).DCTRL as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).ICR as
                                    *mut uint32_t,
                                0xc007ff as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).MASK as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
}
/* *
  * @brief  Initializes the SDIO peripheral according to the specified 
  *         parameters in the SDIO_InitStruct.
  * @param  SDIO_InitStruct : pointer to a SDIO_InitTypeDef structure 
  *         that contains the configuration information for the SDIO peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_Init(mut SDIO_InitStruct:
                                       *mut SDIO_InitTypeDef) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /*---------------------------- SDIO CLKCR Configuration ------------------------*/  
  /* Get the SDIO CLKCR value */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x18000 as libc::c_int as libc::c_uint)
               as *mut SDIO_TypeDef)).CLKCR;
    /* Clear CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, HWFC_EN bits */
    tmpreg &= 0xffff8100 as libc::c_uint;
    /* Set CLKDIV bits according to SDIO_ClockDiv value */
  /* Set PWRSAV bit according to SDIO_ClockPowerSave value */
  /* Set BYPASS bit according to SDIO_ClockBypass value */
  /* Set WIDBUS bits according to SDIO_BusWide value */
  /* Set NEGEDGE bits according to SDIO_ClockEdge value */
  /* Set HWFC_EN bits according to SDIO_HardwareFlowControl value */
    tmpreg |=
        (*SDIO_InitStruct).SDIO_ClockDiv as libc::c_uint |
            (*SDIO_InitStruct).SDIO_ClockPowerSave |
            (*SDIO_InitStruct).SDIO_ClockBypass |
            (*SDIO_InitStruct).SDIO_BusWide |
            (*SDIO_InitStruct).SDIO_ClockEdge |
            (*SDIO_InitStruct).SDIO_HardwareFlowControl;
    /* Write to SDIO CLKCR */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).CLKCR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Fills each SDIO_InitStruct member with its default value.
  * @param  SDIO_InitStruct: pointer to an SDIO_InitTypeDef structure which 
  *   will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_StructInit(mut SDIO_InitStruct:
                                             *mut SDIO_InitTypeDef) {
    /* SDIO_InitStruct members default value */
    (*SDIO_InitStruct).SDIO_ClockDiv = 0 as libc::c_int as uint8_t;
    (*SDIO_InitStruct).SDIO_ClockEdge = 0 as libc::c_int as uint32_t;
    (*SDIO_InitStruct).SDIO_ClockBypass = 0 as libc::c_int as uint32_t;
    (*SDIO_InitStruct).SDIO_ClockPowerSave = 0 as libc::c_int as uint32_t;
    (*SDIO_InitStruct).SDIO_BusWide = 0 as libc::c_int as uint32_t;
    (*SDIO_InitStruct).SDIO_HardwareFlowControl =
        0 as libc::c_int as uint32_t;
}
/* *
  * @brief  Enables or disables the SDIO Clock.
  * @param  NewState: new state of the SDIO Clock. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_ClockCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x18000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0x4
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0x8
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Sets the power status of the controller.
  * @param  SDIO_PowerState: new state of the Power state. 
  *   This parameter can be one of the following values:
  *     @arg SDIO_PowerState_OFF
  *     @arg SDIO_PowerState_ON
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_SetPowerState(mut SDIO_PowerState: uint32_t) {
    /* Check the parameters */
    let ref mut fresh0 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x18000 as libc::c_int as libc::c_uint)
               as *mut SDIO_TypeDef)).POWER;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffffffc as libc::c_uint) as uint32_t
                                    as uint32_t);
    let ref mut fresh1 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x18000 as libc::c_int as libc::c_uint)
               as *mut SDIO_TypeDef)).POWER;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | SDIO_PowerState) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Gets the power status of the controller.
  * @param  None
  * @retval Power status of the controller. The returned value can
  *   be one of the following:
  * - 0x00: Power OFF
  * - 0x02: Power UP
  * - 0x03: Power ON 
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_GetPowerState() -> uint32_t {
    return (*((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x18000 as libc::c_int as
                                              libc::c_uint) as
                  *mut SDIO_TypeDef)).POWER & !(0xfffffffc as libc::c_uint);
}
/* *
  * @brief  Enables or disables the SDIO interrupts.
  * @param  SDIO_IT: specifies the SDIO interrupt sources to be enabled or disabled.
  *   This parameter can be one or a combination of the following values:
  *     @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *     @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *     @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *     @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *     @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *     @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *     @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *     @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *     @arg SDIO_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is zero) interrupt
  *     @arg SDIO_IT_STBITERR: Start bit not detected on all data signals in wide 
  *                            bus mode interrupt
  *     @arg SDIO_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *     @arg SDIO_IT_CMDACT:   Command transfer in progress interrupt
  *     @arg SDIO_IT_TXACT:    Data transmit in progress interrupt
  *     @arg SDIO_IT_RXACT:    Data receive in progress interrupt
  *     @arg SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *     @arg SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *     @arg SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt
  *     @arg SDIO_IT_RXFIFOF:  Receive FIFO full interrupt
  *     @arg SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *     @arg SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt
  *     @arg SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *     @arg SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt
  *     @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  *     @arg SDIO_IT_CEATAEND: CE-ATA command completion signal received for CMD61 interrupt
  * @param  NewState: new state of the specified SDIO interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None 
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_ITConfig(mut SDIO_IT: uint32_t,
                                       mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the SDIO interrupts */
        let ref mut fresh2 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x18000 as libc::c_int as
                                               libc::c_uint) as
                   *mut SDIO_TypeDef)).MASK;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | SDIO_IT) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the SDIO interrupts */
        let ref mut fresh3 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x18000 as libc::c_int as
                                               libc::c_uint) as
                   *mut SDIO_TypeDef)).MASK;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !SDIO_IT) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the SDIO DMA request.
  * @param  NewState: new state of the selected SDIO DMA request.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_DMACmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x18000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0x2c
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0x3
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Initializes the SDIO Command according to the specified 
  *         parameters in the SDIO_CmdInitStruct and send the command.
  * @param  SDIO_CmdInitStruct : pointer to a SDIO_CmdInitTypeDef 
  *         structure that contains the configuration information for the SDIO command.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_SendCommand(mut SDIO_CmdInitStruct:
                                              *mut SDIO_CmdInitTypeDef) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /*---------------------------- SDIO ARG Configuration ------------------------*/
  /* Set the SDIO Argument value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).ARG as
                                    *mut uint32_t,
                                (*SDIO_CmdInitStruct).SDIO_Argument);
    /*---------------------------- SDIO CMD Configuration ------------------------*/  
  /* Get the SDIO CMD value */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x18000 as libc::c_int as libc::c_uint)
               as *mut SDIO_TypeDef)).CMD;
    /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, CPSMEN bits */
    tmpreg &= 0xfffff800 as libc::c_uint;
    /* Set CMDINDEX bits according to SDIO_CmdIndex value */
  /* Set WAITRESP bits according to SDIO_Response value */
  /* Set WAITINT and WAITPEND bits according to SDIO_Wait value */
  /* Set CPSMEN bits according to SDIO_CPSM value */
    tmpreg |=
        (*SDIO_CmdInitStruct).SDIO_CmdIndex |
            (*SDIO_CmdInitStruct).SDIO_Response |
            (*SDIO_CmdInitStruct).SDIO_Wait | (*SDIO_CmdInitStruct).SDIO_CPSM;
    /* Write to SDIO CMD */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).CMD as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Fills each SDIO_CmdInitStruct member with its default value.
  * @param  SDIO_CmdInitStruct: pointer to an SDIO_CmdInitTypeDef 
  *         structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_CmdStructInit(mut SDIO_CmdInitStruct:
                                                *mut SDIO_CmdInitTypeDef) {
    /* SDIO_CmdInitStruct members default value */
    (*SDIO_CmdInitStruct).SDIO_Argument = 0 as libc::c_int as uint32_t;
    (*SDIO_CmdInitStruct).SDIO_CmdIndex = 0 as libc::c_int as uint32_t;
    (*SDIO_CmdInitStruct).SDIO_Response = 0 as libc::c_int as uint32_t;
    (*SDIO_CmdInitStruct).SDIO_Wait = 0 as libc::c_int as uint32_t;
    (*SDIO_CmdInitStruct).SDIO_CPSM = 0 as libc::c_int as uint32_t;
}
/* *
  * @brief  Returns command index of last command for which response received.
  * @param  None
  * @retval Returns the command index of the last command response received.
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_GetCommandResponse() -> uint8_t {
    return (*((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x18000 as libc::c_int as
                                              libc::c_uint) as
                  *mut SDIO_TypeDef)).RESPCMD as uint8_t;
}
/* *
  * @brief  Returns response received from the card for the last command.
  * @param  SDIO_RESP: Specifies the SDIO response register. 
  *   This parameter can be one of the following values:
  *     @arg SDIO_RESP1: Response Register 1
  *     @arg SDIO_RESP2: Response Register 2
  *     @arg SDIO_RESP3: Response Register 3
  *     @arg SDIO_RESP4: Response Register 4
  * @retval The Corresponding response register value.
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_GetResponse(mut SDIO_RESP: uint32_t)
 -> uint32_t {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x18000 as
                                                                libc::c_int as
                                                                libc::c_uint).wrapping_add(0x14
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(SDIO_RESP));
    return *(tmp as *mut uint32_t);
}
/* *
  * @brief  Initializes the SDIO data path according to the specified 
  *   parameters in the SDIO_DataInitStruct.
  * @param  SDIO_DataInitStruct : pointer to a SDIO_DataInitTypeDef structure that
  *   contains the configuration information for the SDIO command.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_DataConfig(mut SDIO_DataInitStruct:
                                             *mut SDIO_DataInitTypeDef) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /*---------------------------- SDIO DTIMER Configuration ---------------------*/
  /* Set the SDIO Data TimeOut value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).DTIMER as
                                    *mut uint32_t,
                                (*SDIO_DataInitStruct).SDIO_DataTimeOut);
    /*---------------------------- SDIO DLEN Configuration -----------------------*/
  /* Set the SDIO DataLength value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).DLEN as
                                    *mut uint32_t,
                                (*SDIO_DataInitStruct).SDIO_DataLength);
    /*---------------------------- SDIO DCTRL Configuration ----------------------*/  
  /* Get the SDIO DCTRL value */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x18000 as libc::c_int as libc::c_uint)
               as *mut SDIO_TypeDef)).DCTRL;
    /* Clear DEN, DTMODE, DTDIR and DBCKSIZE bits */
    tmpreg &= 0xffffff08 as libc::c_uint;
    /* Set DEN bit according to SDIO_DPSM value */
  /* Set DTMODE bit according to SDIO_TransferMode value */
  /* Set DTDIR bit according to SDIO_TransferDir value */
  /* Set DBCKSIZE bits according to SDIO_DataBlockSize value */
    tmpreg |=
        (*SDIO_DataInitStruct).SDIO_DataBlockSize |
            (*SDIO_DataInitStruct).SDIO_TransferDir |
            (*SDIO_DataInitStruct).SDIO_TransferMode |
            (*SDIO_DataInitStruct).SDIO_DPSM;
    /* Write to SDIO DCTRL */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).DCTRL as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Fills each SDIO_DataInitStruct member with its default value.
  * @param  SDIO_DataInitStruct: pointer to an SDIO_DataInitTypeDef structure which
  *         will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_DataStructInit(mut SDIO_DataInitStruct:
                                                 *mut SDIO_DataInitTypeDef) {
    /* SDIO_DataInitStruct members default value */
    (*SDIO_DataInitStruct).SDIO_DataTimeOut = 0xffffffff as libc::c_uint;
    (*SDIO_DataInitStruct).SDIO_DataLength = 0 as libc::c_int as uint32_t;
    (*SDIO_DataInitStruct).SDIO_DataBlockSize = 0 as libc::c_int as uint32_t;
    (*SDIO_DataInitStruct).SDIO_TransferDir = 0 as libc::c_int as uint32_t;
    (*SDIO_DataInitStruct).SDIO_TransferMode = 0 as libc::c_int as uint32_t;
    (*SDIO_DataInitStruct).SDIO_DPSM = 0 as libc::c_int as uint32_t;
}
/* *
  * @brief  Returns number of remaining data bytes to be transferred.
  * @param  None
  * @retval Number of remaining data bytes to be transferred
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_GetDataCounter() -> uint32_t {
    return (*((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x18000 as libc::c_int as
                                              libc::c_uint) as
                  *mut SDIO_TypeDef)).DCOUNT;
}
/* *
  * @brief  Read one data word from Rx FIFO.
  * @param  None
  * @retval Data received
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_ReadData() -> uint32_t {
    return (*((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x18000 as libc::c_int as
                                              libc::c_uint) as
                  *mut SDIO_TypeDef)).FIFO;
}
/* *
  * @brief  Write one data word to Tx FIFO.
  * @param  Data: 32-bit data word to write.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_WriteData(mut Data: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).FIFO as
                                    *mut uint32_t, Data);
}
/* *
  * @brief  Returns the number of words left to be written to or read from FIFO.	
  * @param  None
  * @retval Remaining number of words.
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_GetFIFOCount() -> uint32_t {
    return (*((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x18000 as libc::c_int as
                                              libc::c_uint) as
                  *mut SDIO_TypeDef)).FIFOCNT;
}
/* *
  * @brief  Starts the SD I/O Read Wait operation.	
  * @param  NewState: new state of the Start SDIO Read Wait operation. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_StartSDIOReadWait(mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x18000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0x2c
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0x8
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Stops the SD I/O Read Wait operation.	
  * @param  NewState: new state of the Stop SDIO Read Wait operation. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_StopSDIOReadWait(mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x18000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0x2c
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0x9
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Sets one of the two options of inserting read wait interval.
  * @param  SDIO_ReadWaitMode: SD I/O Read Wait operation mode.
  *   This parameter can be:
  *     @arg SDIO_ReadWaitMode_CLK: Read Wait control by stopping SDIOCLK
  *     @arg SDIO_ReadWaitMode_DATA2: Read Wait control using SDIO_DATA2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_SetSDIOReadWaitMode(mut SDIO_ReadWaitMode:
                                                      uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x18000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0x2c
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0xa
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, SDIO_ReadWaitMode);
}
/* *
  * @brief  Enables or disables the SD I/O Mode Operation.
  * @param  NewState: new state of SDIO specific operation. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_SetSDIOOperation(mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x18000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0x2c
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0xb
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Enables or disables the SD I/O Mode suspend command sending.
  * @param  NewState: new state of the SD I/O Mode suspend command.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_SendSDIOSuspendCmd(mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x18000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0xc
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0xb
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Enables or disables the command completion signal.
  * @param  NewState: new state of command completion signal. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_CommandCompletionCmd(mut NewState:
                                                       FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x18000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0xc
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0xc
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Enables or disables the CE-ATA interrupt.
  * @param  NewState: new state of CE-ATA interrupt. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_CEATAITCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x18000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0xc
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0xd
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t,
                                !(NewState as uint32_t) &
                                    0x1 as libc::c_int as uint32_t);
}
/* *
  * @brief  Sends CE-ATA command (CMD61).
  * @param  NewState: new state of CE-ATA command. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_SendCEATACmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x18000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0xc
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0xe
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Checks whether the specified SDIO flag is set or not.
  * @param  SDIO_FLAG: specifies the flag to check. 
  *   This parameter can be one of the following values:
  *     @arg SDIO_FLAG_CCRCFAIL: Command response received (CRC check failed)
  *     @arg SDIO_FLAG_DCRCFAIL: Data block sent/received (CRC check failed)
  *     @arg SDIO_FLAG_CTIMEOUT: Command response timeout
  *     @arg SDIO_FLAG_DTIMEOUT: Data timeout
  *     @arg SDIO_FLAG_TXUNDERR: Transmit FIFO underrun error
  *     @arg SDIO_FLAG_RXOVERR:  Received FIFO overrun error
  *     @arg SDIO_FLAG_CMDREND:  Command response received (CRC check passed)
  *     @arg SDIO_FLAG_CMDSENT:  Command sent (no response required)
  *     @arg SDIO_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is zero)
  *     @arg SDIO_FLAG_STBITERR: Start bit not detected on all data signals in wide 
  *                              bus mode.
  *     @arg SDIO_FLAG_DBCKEND:  Data block sent/received (CRC check passed)
  *     @arg SDIO_FLAG_CMDACT:   Command transfer in progress
  *     @arg SDIO_FLAG_TXACT:    Data transmit in progress
  *     @arg SDIO_FLAG_RXACT:    Data receive in progress
  *     @arg SDIO_FLAG_TXFIFOHE: Transmit FIFO Half Empty
  *     @arg SDIO_FLAG_RXFIFOHF: Receive FIFO Half Full
  *     @arg SDIO_FLAG_TXFIFOF:  Transmit FIFO full
  *     @arg SDIO_FLAG_RXFIFOF:  Receive FIFO full
  *     @arg SDIO_FLAG_TXFIFOE:  Transmit FIFO empty
  *     @arg SDIO_FLAG_RXFIFOE:  Receive FIFO empty
  *     @arg SDIO_FLAG_TXDAVL:   Data available in transmit FIFO
  *     @arg SDIO_FLAG_RXDAVL:   Data available in receive FIFO
  *     @arg SDIO_FLAG_SDIOIT:   SD I/O interrupt received
  *     @arg SDIO_FLAG_CEATAEND: CE-ATA command completion signal received for CMD61
  * @retval The new state of SDIO_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_GetFlagStatus(mut SDIO_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x18000 as libc::c_int as libc::c_uint)
              as *mut SDIO_TypeDef)).STA & SDIO_FLAG !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Clears the SDIO's pending flags.
  * @param  SDIO_FLAG: specifies the flag to clear.  
  *   This parameter can be one or a combination of the following values:
  *     @arg SDIO_FLAG_CCRCFAIL: Command response received (CRC check failed)
  *     @arg SDIO_FLAG_DCRCFAIL: Data block sent/received (CRC check failed)
  *     @arg SDIO_FLAG_CTIMEOUT: Command response timeout
  *     @arg SDIO_FLAG_DTIMEOUT: Data timeout
  *     @arg SDIO_FLAG_TXUNDERR: Transmit FIFO underrun error
  *     @arg SDIO_FLAG_RXOVERR:  Received FIFO overrun error
  *     @arg SDIO_FLAG_CMDREND:  Command response received (CRC check passed)
  *     @arg SDIO_FLAG_CMDSENT:  Command sent (no response required)
  *     @arg SDIO_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is zero)
  *     @arg SDIO_FLAG_STBITERR: Start bit not detected on all data signals in wide 
  *                              bus mode
  *     @arg SDIO_FLAG_DBCKEND:  Data block sent/received (CRC check passed)
  *     @arg SDIO_FLAG_SDIOIT:   SD I/O interrupt received
  *     @arg SDIO_FLAG_CEATAEND: CE-ATA command completion signal received for CMD61
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_ClearFlag(mut SDIO_FLAG: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).ICR as
                                    *mut uint32_t, SDIO_FLAG);
}
/* *
  * @brief  Checks whether the specified SDIO interrupt has occurred or not.
  * @param  SDIO_IT: specifies the SDIO interrupt source to check. 
  *   This parameter can be one of the following values:
  *     @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *     @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *     @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *     @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *     @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *     @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *     @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *     @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *     @arg SDIO_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is zero) interrupt
  *     @arg SDIO_IT_STBITERR: Start bit not detected on all data signals in wide 
  *                            bus mode interrupt
  *     @arg SDIO_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *     @arg SDIO_IT_CMDACT:   Command transfer in progress interrupt
  *     @arg SDIO_IT_TXACT:    Data transmit in progress interrupt
  *     @arg SDIO_IT_RXACT:    Data receive in progress interrupt
  *     @arg SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *     @arg SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *     @arg SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt
  *     @arg SDIO_IT_RXFIFOF:  Receive FIFO full interrupt
  *     @arg SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *     @arg SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt
  *     @arg SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *     @arg SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt
  *     @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  *     @arg SDIO_IT_CEATAEND: CE-ATA command completion signal received for CMD61 interrupt
  * @retval The new state of SDIO_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_GetITStatus(mut SDIO_IT: uint32_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    /* Check the parameters */
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x18000 as libc::c_int as libc::c_uint)
              as *mut SDIO_TypeDef)).STA & SDIO_IT !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_sdio.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the SDIO firmware
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
/* * @addtogroup SDIO
  * @{
  */
/* * @defgroup SDIO_Exported_Types
  * @{
  */
/* !< Specifies the clock transition on which the bit capture is made.
                                           This parameter can be a value of @ref SDIO_Clock_Edge */
/* !< Specifies whether the SDIO Clock divider bypass is
                                           enabled or disabled.
                                           This parameter can be a value of @ref SDIO_Clock_Bypass */
/* !< Specifies whether SDIO Clock output is enabled or
                                           disabled when the bus is idle.
                                           This parameter can be a value of @ref SDIO_Clock_Power_Save */
/* !< Specifies the SDIO bus width.
                                           This parameter can be a value of @ref SDIO_Bus_Wide */
/* !< Specifies whether the SDIO hardware flow control is enabled or disabled.
                                           This parameter can be a value of @ref SDIO_Hardware_Flow_Control */
/* !< Specifies the clock frequency of the SDIO controller.
                                           This parameter can be a value between 0x00 and 0xFF. */
/* !< Specifies the SDIO command argument which is sent
                                to a card as part of a command message. If a command
                                contains an argument, it must be loaded into this register
                                before writing the command to the command register */
/* !< Specifies the SDIO command index. It must be lower than 0x40. */
/* !< Specifies the SDIO response type.
                                This parameter can be a value of @ref SDIO_Response_Type */
/* !< Specifies whether SDIO wait-for-interrupt request is enabled or disabled.
                                This parameter can be a value of @ref SDIO_Wait_Interrupt_State */
/* !< Specifies whether SDIO Command path state machine (CPSM)
                                is enabled or disabled.
                                This parameter can be a value of @ref SDIO_CPSM_State */
/* !< Specifies the data timeout period in card bus clock periods. */
/* !< Specifies the number of data bytes to be transferred. */
/* !< Specifies the data block size for block transfer.
                                     This parameter can be a value of @ref SDIO_Data_Block_Size */
/* !< Specifies the data transfer direction, whether the transfer
                                     is a read or write.
                                     This parameter can be a value of @ref SDIO_Transfer_Direction */
/* !< Specifies whether data transfer is in stream or block mode.
                                     This parameter can be a value of @ref SDIO_Transfer_Type */
/* !< Specifies whether SDIO Data path state machine (DPSM)
                                     is enabled or disabled.
                                     This parameter can be a value of @ref SDIO_DPSM_State */
/* *
  * @}
  */
/* * @defgroup SDIO_Exported_Constants
  * @{
  */
/* * @defgroup SDIO_Clock_Edge 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Clock_Bypass 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Clock_Power_Save 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Bus_Wide 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Hardware_Flow_Control 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Power_State 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Interrupt_sources 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Command_Index
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Response_Type 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Wait_Interrupt_State 
  * @{
  */
/* !< SDIO No Wait, TimeOut is enabled */
/* !< SDIO Wait Interrupt Request */
/* !< SDIO Wait End of transfer */
/* *
  * @}
  */
/* * @defgroup SDIO_CPSM_State 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Response_Registers 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Data_Length 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Data_Block_Size 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Transfer_Direction 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Transfer_Type 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_DPSM_State 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Flags 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Read_Wait_Mode 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SDIO_Exported_Functions
  * @{
  */
/* *
  * @brief  Clears the SDIO's interrupt pending bits.
  * @param  SDIO_IT: specifies the interrupt pending bit to clear. 
  *   This parameter can be one or a combination of the following values:
  *     @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *     @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *     @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *     @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *     @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *     @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *     @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *     @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *     @arg SDIO_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is zero) interrupt
  *     @arg SDIO_IT_STBITERR: Start bit not detected on all data signals in wide 
  *                            bus mode interrupt
  *     @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  *     @arg SDIO_IT_CEATAEND: CE-ATA command completion signal received for CMD61
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SDIO_ClearITPendingBit(mut SDIO_IT: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x18000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut SDIO_TypeDef)).ICR as
                                    *mut uint32_t, SDIO_IT);
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
