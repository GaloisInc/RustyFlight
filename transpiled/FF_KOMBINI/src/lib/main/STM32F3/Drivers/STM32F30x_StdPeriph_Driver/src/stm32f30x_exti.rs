use ::libc;
pub type __uint32_t = libc::c_uint;
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
    pub RESERVED1: uint32_t,
    pub RESERVED2: uint32_t,
    pub IMR2: uint32_t,
    pub EMR2: uint32_t,
    pub RTSR2: uint32_t,
    pub FTSR2: uint32_t,
    pub SWIER2: uint32_t,
    pub PR2: uint32_t,
}
pub type EXTIMode_TypeDef = libc::c_uint;
pub const EXTI_Mode_Event: EXTIMode_TypeDef = 4;
pub const EXTI_Mode_Interrupt: EXTIMode_TypeDef = 0;
pub type EXTITrigger_TypeDef = libc::c_uint;
pub const EXTI_Trigger_Rising_Falling: EXTITrigger_TypeDef = 16;
pub const EXTI_Trigger_Falling: EXTITrigger_TypeDef = 12;
pub const EXTI_Trigger_Rising: EXTITrigger_TypeDef = 8;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EXTI_InitTypeDef {
    pub EXTI_Line: uint32_t,
    pub EXTI_Mode: EXTIMode_TypeDef,
    pub EXTI_Trigger: EXTITrigger_TypeDef,
    pub EXTI_LineCmd: FunctionalState,
}
/* No interrupt selected */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup EXTI_Private_Functions 
  * @{
  */
/* * @defgroup EXTI_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and Configuration functions #####
 ===============================================================================

@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the EXTI peripheral registers to their default reset 
  *         values.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_DeInit() {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).IMR as
                                    *mut uint32_t,
                                0x1f800000 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).EMR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).RTSR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).FTSR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).SWIER as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).PR as
                                    *mut uint32_t,
                                0xe07fffff as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).IMR2 as
                                    *mut uint32_t,
                                0xc as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).EMR2 as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).RTSR2 as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).FTSR2 as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).SWIER2 as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).PR2 as
                                    *mut uint32_t,
                                0x3 as libc::c_int as uint32_t);
}
/* *
  * @brief  Initializes the EXTI peripheral according to the specified
  *         parameters in the EXTI_InitStruct.
  *    EXTI_Line specifies the EXTI line (EXTI0....EXTI35).
  *    EXTI_Mode specifies which EXTI line is used as interrupt or an event.
  *    EXTI_Trigger selects the trigger. When the trigger occurs, interrupt
  *                 pending bit will be set.
  *    EXTI_LineCmd controls (Enable/Disable) the EXTI line.
  * @param  EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure that 
  *         contains the configuration information for the EXTI peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_Init(mut EXTI_InitStruct:
                                       *mut EXTI_InitTypeDef) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmp =
        (0x40000000 as libc::c_int as
             uint32_t).wrapping_add(0x10000 as libc::c_int as
                                        libc::c_uint).wrapping_add(0x400 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint);
    if (*EXTI_InitStruct).EXTI_LineCmd as libc::c_uint !=
           DISABLE as libc::c_int as libc::c_uint {
        /* Clear EXTI line configuration */
        let ref mut fresh0 =
            *((&mut (*((0x40000000 as libc::c_int as
                            uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                       libc::c_uint).wrapping_add(0x400
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                           as *mut EXTI_TypeDef)).IMR as *mut uint32_t as
                   uint32_t).wrapping_add(((*EXTI_InitStruct).EXTI_Line >>
                                               5 as
                                                   libc::c_int).wrapping_mul(0x20
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint))
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((1 as libc::c_int) <<
                                                ((*EXTI_InitStruct).EXTI_Line
                                                     &
                                                     0x1f as libc::c_int as
                                                         libc::c_uint)) as
                                               uint32_t)) as uint32_t as
                                        uint32_t);
        let ref mut fresh1 =
            *((&mut (*((0x40000000 as libc::c_int as
                            uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                       libc::c_uint).wrapping_add(0x400
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                           as *mut EXTI_TypeDef)).EMR as *mut uint32_t as
                   uint32_t).wrapping_add(((*EXTI_InitStruct).EXTI_Line >>
                                               5 as
                                                   libc::c_int).wrapping_mul(0x20
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint))
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((1 as libc::c_int) <<
                                                ((*EXTI_InitStruct).EXTI_Line
                                                     &
                                                     0x1f as libc::c_int as
                                                         libc::c_uint)) as
                                               uint32_t)) as uint32_t as
                                        uint32_t);
        tmp =
            (tmp as
                 libc::c_uint).wrapping_add(((*EXTI_InitStruct).EXTI_Mode as
                                                 libc::c_uint).wrapping_add(((*EXTI_InitStruct).EXTI_Line
                                                                                 >>
                                                                                 5
                                                                                     as
                                                                                     libc::c_int).wrapping_mul(0x20
                                                                                                                   as
                                                                                                                   libc::c_int
                                                                                                                   as
                                                                                                                   libc::c_uint)))
                as uint32_t as uint32_t;
        let ref mut fresh2 = *(tmp as *mut uint32_t);
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((1 as libc::c_int) <<
                                              ((*EXTI_InitStruct).EXTI_Line &
                                                   0x1f as libc::c_int as
                                                       libc::c_uint)) as
                                             uint32_t) as uint32_t as
                                        uint32_t);
        tmp =
            (0x40000000 as libc::c_int as
                 uint32_t).wrapping_add(0x10000 as libc::c_int as
                                            libc::c_uint).wrapping_add(0x400
                                                                           as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint);
        /* Clear Rising Falling edge configuration */
        let ref mut fresh3 =
            *((&mut (*((0x40000000 as libc::c_int as
                            uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                       libc::c_uint).wrapping_add(0x400
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                           as *mut EXTI_TypeDef)).RTSR as *mut uint32_t as
                   uint32_t).wrapping_add(((*EXTI_InitStruct).EXTI_Line >>
                                               5 as
                                                   libc::c_int).wrapping_mul(0x20
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint))
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((1 as libc::c_int) <<
                                                ((*EXTI_InitStruct).EXTI_Line
                                                     &
                                                     0x1f as libc::c_int as
                                                         libc::c_uint)) as
                                               uint32_t)) as uint32_t as
                                        uint32_t);
        let ref mut fresh4 =
            *((&mut (*((0x40000000 as libc::c_int as
                            uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                       libc::c_uint).wrapping_add(0x400
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                           as *mut EXTI_TypeDef)).FTSR as *mut uint32_t as
                   uint32_t).wrapping_add(((*EXTI_InitStruct).EXTI_Line >>
                                               5 as
                                                   libc::c_int).wrapping_mul(0x20
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint))
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((1 as libc::c_int) <<
                                                ((*EXTI_InitStruct).EXTI_Line
                                                     &
                                                     0x1f as libc::c_int as
                                                         libc::c_uint)) as
                                               uint32_t)) as uint32_t as
                                        uint32_t);
        /* Select the trigger for the selected interrupts */
        if (*EXTI_InitStruct).EXTI_Trigger as libc::c_uint ==
               EXTI_Trigger_Rising_Falling as libc::c_int as libc::c_uint {
            /* Rising Falling edge */
            let ref mut fresh5 =
                *((&mut (*((0x40000000 as libc::c_int as
                                uint32_t).wrapping_add(0x10000 as libc::c_int
                                                           as
                                                           libc::c_uint).wrapping_add(0x400
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                               as *mut EXTI_TypeDef)).RTSR as *mut uint32_t as
                       uint32_t).wrapping_add(((*EXTI_InitStruct).EXTI_Line >>
                                                   5 as
                                                       libc::c_int).wrapping_mul(0x20
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     libc::c_uint))
                      as *mut uint32_t);
            ::core::ptr::write_volatile(fresh5,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             ((1 as libc::c_int) <<
                                                  ((*EXTI_InitStruct).EXTI_Line
                                                       &
                                                       0x1f as libc::c_int as
                                                           libc::c_uint)) as
                                                 uint32_t) as uint32_t as
                                            uint32_t);
            let ref mut fresh6 =
                *((&mut (*((0x40000000 as libc::c_int as
                                uint32_t).wrapping_add(0x10000 as libc::c_int
                                                           as
                                                           libc::c_uint).wrapping_add(0x400
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                               as *mut EXTI_TypeDef)).FTSR as *mut uint32_t as
                       uint32_t).wrapping_add(((*EXTI_InitStruct).EXTI_Line >>
                                                   5 as
                                                       libc::c_int).wrapping_mul(0x20
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     libc::c_uint))
                      as *mut uint32_t);
            ::core::ptr::write_volatile(fresh6,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             ((1 as libc::c_int) <<
                                                  ((*EXTI_InitStruct).EXTI_Line
                                                       &
                                                       0x1f as libc::c_int as
                                                           libc::c_uint)) as
                                                 uint32_t) as uint32_t as
                                            uint32_t)
        } else {
            tmp =
                (tmp as
                     libc::c_uint).wrapping_add(((*EXTI_InitStruct).EXTI_Trigger
                                                     as
                                                     libc::c_uint).wrapping_add(((*EXTI_InitStruct).EXTI_Line
                                                                                     >>
                                                                                     5
                                                                                         as
                                                                                         libc::c_int).wrapping_mul(0x20
                                                                                                                       as
                                                                                                                       libc::c_int
                                                                                                                       as
                                                                                                                       libc::c_uint)))
                    as uint32_t as uint32_t;
            let ref mut fresh7 = *(tmp as *mut uint32_t);
            ::core::ptr::write_volatile(fresh7,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             ((1 as libc::c_int) <<
                                                  ((*EXTI_InitStruct).EXTI_Line
                                                       &
                                                       0x1f as libc::c_int as
                                                           libc::c_uint)) as
                                                 uint32_t) as uint32_t as
                                            uint32_t)
        }
    } else {
        tmp =
            (tmp as
                 libc::c_uint).wrapping_add(((*EXTI_InitStruct).EXTI_Mode as
                                                 libc::c_uint).wrapping_add(((*EXTI_InitStruct).EXTI_Line
                                                                                 >>
                                                                                 5
                                                                                     as
                                                                                     libc::c_int).wrapping_mul(0x20
                                                                                                                   as
                                                                                                                   libc::c_int
                                                                                                                   as
                                                                                                                   libc::c_uint)))
                as uint32_t as uint32_t;
        /* Disable the selected external lines */
        let ref mut fresh8 = *(tmp as *mut uint32_t);
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((1 as libc::c_int) <<
                                                ((*EXTI_InitStruct).EXTI_Line
                                                     &
                                                     0x1f as libc::c_int as
                                                         libc::c_uint)) as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Fills each EXTI_InitStruct member with its reset value.
  * @param  EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_StructInit(mut EXTI_InitStruct:
                                             *mut EXTI_InitTypeDef) {
    (*EXTI_InitStruct).EXTI_Line = 0 as libc::c_int as uint32_t;
    (*EXTI_InitStruct).EXTI_Mode = EXTI_Mode_Interrupt;
    (*EXTI_InitStruct).EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    (*EXTI_InitStruct).EXTI_LineCmd = DISABLE;
}
/* *
  * @brief  Generates a Software interrupt on selected EXTI line.
  * @param  EXTI_Line: specifies the EXTI line on which the software interrupt
  *         will be generated.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..20).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_GenerateSWInterrupt(mut EXTI_Line: uint32_t) {
    /* Check the parameters */
    let ref mut fresh9 =
        *((&mut (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut EXTI_TypeDef)).SWIER as *mut uint32_t as
               uint32_t).wrapping_add((EXTI_Line >>
                                           5 as
                                               libc::c_int).wrapping_mul(0x20
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint))
              as *mut uint32_t);
    ::core::ptr::write_volatile(fresh9,
                                (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((1 as libc::c_int) <<
                                          (EXTI_Line &
                                               0x1f as libc::c_int as
                                                   libc::c_uint)) as uint32_t)
                                    as uint32_t as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup EXTI_Group2 Interrupts and flags management functions
 *  @brief    EXTI Interrupts and flags management functions
 *
@verbatim  
 ===============================================================================
              ##### Interrupts and flags management functions #####
 ===============================================================================
    [..]
    This section provides functions allowing to configure the EXTI Interrupts 
    sources and check or clear the flags or pending bits status.
    
@endverbatim
  * @{
  */
/* *
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  EXTI_Line: specifies the EXTI line flag to check.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..20).
  * @retval The new state of EXTI_Line (SET or RESET).                  
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_GetFlagStatus(mut EXTI_Line: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if *((&mut (*((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x400
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut EXTI_TypeDef)).PR as *mut uint32_t as
              uint32_t).wrapping_add((EXTI_Line >>
                                          5 as
                                              libc::c_int).wrapping_mul(0x20
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint))
             as *mut uint32_t) &
           ((1 as libc::c_int) <<
                (EXTI_Line & 0x1f as libc::c_int as libc::c_uint)) as uint32_t
           != RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Clears the EXTI's line pending flags.
  * @param  EXTI_Line: specifies the EXTI lines flags to clear.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..20).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_ClearFlag(mut EXTI_Line: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile((&mut (*((0x40000000 as libc::c_int as
                                              uint32_t).wrapping_add(0x10000
                                                                         as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x400
                                                                                                        as
                                                                                                        libc::c_int
                                                                                                        as
                                                                                                        libc::c_uint)
                                             as *mut EXTI_TypeDef)).PR as
                                     *mut uint32_t as
                                     uint32_t).wrapping_add((EXTI_Line >>
                                                                 5 as
                                                                     libc::c_int).wrapping_mul(0x20
                                                                                                   as
                                                                                                   libc::c_int
                                                                                                   as
                                                                                                   libc::c_uint))
                                    as *mut uint32_t,
                                ((1 as libc::c_int) <<
                                     (EXTI_Line &
                                          0x1f as libc::c_int as
                                              libc::c_uint)) as uint32_t);
}
/* *
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..20).
  * @retval The new state of EXTI_Line (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_GetITStatus(mut EXTI_Line: uint32_t)
 -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    /* Check the parameters */
    if *((&mut (*((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x400
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut EXTI_TypeDef)).PR as *mut uint32_t as
              uint32_t).wrapping_add((EXTI_Line >>
                                          5 as
                                              libc::c_int).wrapping_mul(0x20
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint))
             as *mut uint32_t) &
           ((1 as libc::c_int) <<
                (EXTI_Line & 0x1f as libc::c_int as libc::c_uint)) as uint32_t
           != RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f30x_exti.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the EXTI 
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
/* * @addtogroup EXTI
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  EXTI mode enumeration  
  */
/* * 
  * @brief  EXTI Trigger enumeration  
  */
/* * 
  * @brief  EXTI Init Structure definition  
  */
/* !< Specifies the EXTI lines to be enabled or disabled.
                                         This parameter can be any combination of @ref EXTI_Lines */
/* !< Specifies the mode for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */
/* !< Specifies the trigger signal active edge for the EXTI lines.
                                         This parameter can be a value of @ref EXTITrigger_TypeDef */
/* !< Specifies the new state of the selected EXTI lines.
                                         This parameter can be set either to ENABLE or DISABLE */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup EXTI_Exported_Constants
  * @{
  */ 
/* * @defgroup EXTI_Lines 
  * @{
  */
/* !< External interrupt line 0  */
/* !< External interrupt line 1  */
/* !< External interrupt line 2  */
/* !< External interrupt line 3  */
/* !< External interrupt line 4  */
/* !< External interrupt line 5  */
/* !< External interrupt line 6  */
/* !< External interrupt line 7  */
/* !< External interrupt line 8  */
/* !< External interrupt line 9  */
/* !< External interrupt line 10 */
/* !< External interrupt line 11 */
/* !< External interrupt line 12 */
/* !< External interrupt line 13 */
/* !< External interrupt line 14 */
/* !< External interrupt line 15 */
/* !< External interrupt line 16 
                                                      Connected to the PVD Output */
/* !< Internal interrupt line 17 
                                                      Connected to the RTC Alarm 
                                                      event */
/* !< Internal interrupt line 18 
                                                      Connected to the USB Device
                                                      Wakeup from suspend event */
/* !< Internal interrupt line 19
                                                      Connected to the RTC Tamper
                                                      and Time Stamp events */
/* !< Internal interrupt line 20
                                                      Connected to the RTC wakeup
                                                      event */
/* !< Internal interrupt line 21
                                                      Connected to the Comparator 1
                                                      event */
/* !< Internal interrupt line 22
                                                      Connected to the Comparator 2
                                                      event */
/* !< Internal interrupt line 23
                                                      Connected to the I2C1 wakeup
                                                      event */
/* !< Internal interrupt line 24
                                                      Connected to the I2C2 wakeup
                                                      event */
/* !< Internal interrupt line 25
                                                      Connected to the USART1 wakeup
                                                      event */
/* !< Internal interrupt line 26
                                                      Connected to the USART2 wakeup
                                                      event */
/* !< Internal interrupt line 27
                                                       reserved */
/* !< Internal interrupt line 28
                                                      Connected to the USART3 wakeup
                                                      event */
/* !< Internal interrupt line 29
                                                      Connected to the Comparator 3 
                                                      event */
/* !< Internal interrupt line 30
                                                      Connected to the Comparator 4 
                                                      event */
/* !< Internal interrupt line 31
                                                      Connected to the Comparator 5 
                                                      event */
/* !< Internal interrupt line 32
                                                      Connected to the Comparator 6 
                                                      event */
/* !< Internal interrupt line 33
                                                      Connected to the Comparator 7 
                                                      event */
/* !< Internal interrupt line 34
                                                      Connected to the USART4 wakeup
                                                      event */
/* !< Internal interrupt line 35
                                                      Connected to the USART5 wakeup
                                                      event */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Function used to set the EXTI configuration to the default reset state *****/
/* Initialization and Configuration functions *********************************/
/* Interrupts and flags management functions **********************************/
/* *
  * @brief  Clears the EXTI's line pending bits.
  * @param  EXTI_Line: specifies the EXTI lines to clear.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..20).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_ClearITPendingBit(mut EXTI_Line: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile((&mut (*((0x40000000 as libc::c_int as
                                              uint32_t).wrapping_add(0x10000
                                                                         as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_add(0x400
                                                                                                        as
                                                                                                        libc::c_int
                                                                                                        as
                                                                                                        libc::c_uint)
                                             as *mut EXTI_TypeDef)).PR as
                                     *mut uint32_t as
                                     uint32_t).wrapping_add((EXTI_Line >>
                                                                 5 as
                                                                     libc::c_int).wrapping_mul(0x20
                                                                                                   as
                                                                                                   libc::c_int
                                                                                                   as
                                                                                                   libc::c_uint))
                                    as *mut uint32_t,
                                ((1 as libc::c_int) <<
                                     (EXTI_Line &
                                          0x1f as libc::c_int as
                                              libc::c_uint)) as uint32_t);
}
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
/* *
  * @}
  */
