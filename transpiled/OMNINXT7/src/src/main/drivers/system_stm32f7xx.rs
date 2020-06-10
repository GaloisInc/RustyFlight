use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_RCC_GetHCLKFreq() -> uint32_t;
    /* *
  * @}
  */
    /* __MPU_PRESENT */
    /* *
  * @}
  */
    /* Exported constants --------------------------------------------------------*/
    /* * @defgroup CORTEX_Exported_Constants CORTEX Exported Constants
  * @{
  */
    /* * @defgroup CORTEX_Preemption_Priority_Group CORTEX Preemption Priority Group
  * @{
  */
    /* !< 0 bits for pre-emption priority
                                                                 4 bits for subpriority */
    /* !< 1 bits for pre-emption priority
                                                                 3 bits for subpriority */
    /* !< 2 bits for pre-emption priority
                                                                 2 bits for subpriority */
    /* !< 3 bits for pre-emption priority
                                                                 1 bits for subpriority */
    /* !< 4 bits for pre-emption priority
                                                                 0 bits for subpriority */
    /* *
  * @}
  */
    /* * @defgroup CORTEX_SysTick_clock_source CORTEX _SysTick clock source 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup CORTEX_MPU_HFNMI_PRIVDEF_Control MPU HFNMI and PRIVILEGED Access control
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup CORTEX_MPU_Region_Enable CORTEX MPU Region Enable
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup CORTEX_MPU_Instruction_Access CORTEX MPU Instruction Access
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup CORTEX_MPU_Access_Shareable CORTEX MPU Instruction Access Shareable
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup CORTEX_MPU_Access_Cacheable CORTEX MPU Instruction Access Cacheable
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup CORTEX_MPU_Access_Bufferable CORTEX MPU Instruction Access Bufferable
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup CORTEX_MPU_TEX_Levels MPU TEX Levels
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup CORTEX_MPU_Region_Size CORTEX MPU Region Size
  * @{
  */
    /* *                                
  * @}
  */
    /* * @defgroup CORTEX_MPU_Region_Permission_Attributes CORTEX MPU Region Permission Attributes 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup CORTEX_MPU_Region_Number CORTEX MPU Region Number
  * @{
  */
    /* *
  * @}
  */
    /* __MPU_PRESENT */
    /* *
  * @}
  */
    /* Exported Macros -----------------------------------------------------------*/
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup CORTEX_Exported_Functions
  * @{
  */
    /* * @addtogroup CORTEX_Exported_Functions_Group1
 * @{
 */
/* Initialization and de-initialization functions *****************************/
    #[no_mangle]
    fn HAL_NVIC_SetPriorityGrouping(PriorityGroup: uint32_t);
    #[no_mangle]
    fn HAL_SYSTICK_Config(TicksNumb: uint32_t) -> uint32_t;
    #[no_mangle]
    fn HAL_SYSTICK_CLKSourceConfig(CLKSource: uint32_t);
    #[no_mangle]
    fn HAL_PWR_EnableBkUpAccess();
    #[no_mangle]
    static mut mpuResetFn: mpuResetFnPtr;
    #[no_mangle]
    fn cycleCounterInit();
    #[no_mangle]
    static mut cachedRccCsrValue: uint32_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
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
    pub SHPR: [uint8_t; 12],
    pub SHCSR: uint32_t,
    pub CFSR: uint32_t,
    pub HFSR: uint32_t,
    pub DFSR: uint32_t,
    pub MMFAR: uint32_t,
    pub BFAR: uint32_t,
    pub AFSR: uint32_t,
    pub ID_PFR: [uint32_t; 2],
    pub ID_DFR: uint32_t,
    pub ID_AFR: uint32_t,
    pub ID_MFR: [uint32_t; 4],
    pub ID_ISAR: [uint32_t; 5],
    pub RESERVED0: [uint32_t; 1],
    pub CLIDR: uint32_t,
    pub CTR: uint32_t,
    pub CCSIDR: uint32_t,
    pub CSSELR: uint32_t,
    pub CPACR: uint32_t,
    pub RESERVED3: [uint32_t; 93],
    pub STIR: uint32_t,
    pub RESERVED4: [uint32_t; 15],
    pub MVFR0: uint32_t,
    pub MVFR1: uint32_t,
    pub MVFR2: uint32_t,
    pub RESERVED5: [uint32_t; 1],
    pub ICIALLU: uint32_t,
    pub RESERVED6: [uint32_t; 1],
    pub ICIMVAU: uint32_t,
    pub DCIMVAC: uint32_t,
    pub DCISW: uint32_t,
    pub DCCMVAU: uint32_t,
    pub DCCMVAC: uint32_t,
    pub DCCSW: uint32_t,
    pub DCCIMVAC: uint32_t,
    pub DCCISW: uint32_t,
    pub RESERVED7: [uint32_t; 6],
    pub ITCMCR: uint32_t,
    pub DTCMCR: uint32_t,
    pub AHBPCR: uint32_t,
    pub CACR: uint32_t,
    pub AHBSCR: uint32_t,
    pub RESERVED8: [uint32_t; 1],
    pub ABFSR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct MPU_Type {
    pub TYPE: uint32_t,
    pub CTRL: uint32_t,
    pub RNR: uint32_t,
    pub RBAR: uint32_t,
    pub RASR: uint32_t,
    pub RBAR_A1: uint32_t,
    pub RASR_A1: uint32_t,
    pub RBAR_A2: uint32_t,
    pub RASR_A2: uint32_t,
    pub RBAR_A3: uint32_t,
    pub RASR_A3: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SYSCFG_TypeDef {
    pub MEMRMP: uint32_t,
    pub PMC: uint32_t,
    pub EXTICR: [uint32_t; 4],
    pub RESERVED: [uint32_t; 2],
    pub CMPCR: uint32_t,
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
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
//#define DEBUG_MPU_DATA_READY_INTERRUPT
// MPU6050
// MPU3050, 6000 and 6050
// RA = Register Address
//[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
//[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
//[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
//[7:0] X_FINE_GAIN
//[7:0] Y_FINE_GAIN
//[7:0] Z_FINE_GAIN
//[15:0] XA_OFFS
//[15:0] YA_OFFS
//[15:0] ZA_OFFS
// Product ID Register
//[15:0] XG_OFFS_USR
//[15:0] YG_OFFS_USR
//[15:0] ZG_OFFS_USR
// RF = Register Flag
pub type mpuResetFnPtr = Option<unsafe extern "C" fn() -> ()>;
#[inline(always)]
unsafe extern "C" fn __disable_irq() {
    asm!("cpsid i" : : : "memory" : "volatile");
}
#[inline(always)]
unsafe extern "C" fn __set_MSP(mut topOfMainStack: uint32_t) {
    asm!("MSR msp, $0" : : "r" (topOfMainStack) : : "volatile");
}
#[inline(always)]
unsafe extern "C" fn __ISB() { asm!("isb 0xF" : : : "memory" : "volatile"); }
#[inline(always)]
unsafe extern "C" fn __DSB() { asm!("dsb 0xF" : : : "memory" : "volatile"); }
#[inline]
unsafe extern "C" fn __NVIC_SystemReset() {
    __DSB();
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).AIRCR as
                                    *mut uint32_t,
                                ((0x5fa as libc::c_ulong) <<
                                     16 as libc::c_uint |
                                     (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).AIRCR as
                                         libc::c_ulong &
                                         (7 as libc::c_ulong) <<
                                             8 as libc::c_uint |
                                     (1 as libc::c_ulong) <<
                                         2 as libc::c_uint) as uint32_t);
    __DSB();
    loop  { asm!("nop" : : : : "volatile") };
}
#[inline]
unsafe extern "C" fn SCB_CleanDCache_by_Addr(mut addr: *mut uint32_t,
                                             mut dsize: int32_t) {
    let mut op_size: int32_t = dsize;
    let mut op_addr: uint32_t = addr as uint32_t;
    let mut linesize: int32_t = 32 as libc::c_int;
    __DSB();
    while op_size > 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0xd00
                                                                                 as
                                                                                 libc::c_ulong)
                                                as *mut SCB_Type)).DCCMVAC as
                                        *mut uint32_t, op_addr);
        op_addr =
            (op_addr as libc::c_uint).wrapping_add(linesize as uint32_t) as
                uint32_t as uint32_t;
        op_size -= linesize
    }
    __DSB();
    __ISB();
}
/* *
  * @}
  */
/* * @defgroup CORTEX_LL_EF_MPU MPU
  * @{
  */
/* *
  * @brief  Enable MPU with input options
  * @rmtoll MPU_CTRL     ENABLE        LL_MPU_Enable
  * @param  Options This parameter can be one of the following values:
  *         @arg @ref LL_MPU_CTRL_HFNMI_PRIVDEF_NONE
  *         @arg @ref LL_MPU_CTRL_HARDFAULT_NMI
  *         @arg @ref LL_MPU_CTRL_PRIVILEGED_DEFAULT
  *         @arg @ref LL_MPU_CTRL_HFNMI_PRIVDEF
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_MPU_Enable(mut Options: uint32_t) {
    /* Enable the MPU*/
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd90
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut MPU_Type)).CTRL as
                                    *mut uint32_t,
                                (1 as libc::c_ulong |
                                     Options as libc::c_ulong) as uint32_t);
    /* Ensure MPU settings take effects */
    __DSB();
    /* Sequence instruction fetches using update settings */
    __ISB();
}
/* *
  * @brief  Configure and enable a region
  * @rmtoll MPU_RNR      REGION        LL_MPU_ConfigRegion\n
  *         MPU_RBAR     REGION        LL_MPU_ConfigRegion\n
  *         MPU_RBAR     ADDR          LL_MPU_ConfigRegion\n
  *         MPU_RASR     XN            LL_MPU_ConfigRegion\n
  *         MPU_RASR     AP            LL_MPU_ConfigRegion\n
  *         MPU_RASR     S             LL_MPU_ConfigRegion\n
  *         MPU_RASR     C             LL_MPU_ConfigRegion\n
  *         MPU_RASR     B             LL_MPU_ConfigRegion\n
  *         MPU_RASR     SIZE          LL_MPU_ConfigRegion
  * @param  Region This parameter can be one of the following values:
  *         @arg @ref LL_MPU_REGION_NUMBER0
  *         @arg @ref LL_MPU_REGION_NUMBER1
  *         @arg @ref LL_MPU_REGION_NUMBER2
  *         @arg @ref LL_MPU_REGION_NUMBER3
  *         @arg @ref LL_MPU_REGION_NUMBER4
  *         @arg @ref LL_MPU_REGION_NUMBER5
  *         @arg @ref LL_MPU_REGION_NUMBER6
  *         @arg @ref LL_MPU_REGION_NUMBER7
  * @param  Address Value of region base address
  * @param  SubRegionDisable Sub-region disable value between Min_Data = 0x00 and Max_Data = 0xFF
  * @param  Attributes This parameter can be a combination of the following values:
  *         @arg @ref LL_MPU_REGION_SIZE_32B or @ref LL_MPU_REGION_SIZE_64B or @ref LL_MPU_REGION_SIZE_128B or @ref LL_MPU_REGION_SIZE_256B or @ref LL_MPU_REGION_SIZE_512B
  *           or @ref LL_MPU_REGION_SIZE_1KB or @ref LL_MPU_REGION_SIZE_2KB or @ref LL_MPU_REGION_SIZE_4KB or @ref LL_MPU_REGION_SIZE_8KB or @ref LL_MPU_REGION_SIZE_16KB
  *           or @ref LL_MPU_REGION_SIZE_32KB or @ref LL_MPU_REGION_SIZE_64KB or @ref LL_MPU_REGION_SIZE_128KB or @ref LL_MPU_REGION_SIZE_256KB or @ref LL_MPU_REGION_SIZE_512KB
  *           or @ref LL_MPU_REGION_SIZE_1MB or @ref LL_MPU_REGION_SIZE_2MB or @ref LL_MPU_REGION_SIZE_4MB or @ref LL_MPU_REGION_SIZE_8MB or @ref LL_MPU_REGION_SIZE_16MB
  *           or @ref LL_MPU_REGION_SIZE_32MB or @ref LL_MPU_REGION_SIZE_64MB or @ref LL_MPU_REGION_SIZE_128MB or @ref LL_MPU_REGION_SIZE_256MB or @ref LL_MPU_REGION_SIZE_512MB
  *           or @ref LL_MPU_REGION_SIZE_1GB or @ref LL_MPU_REGION_SIZE_2GB or @ref LL_MPU_REGION_SIZE_4GB
  *         @arg @ref LL_MPU_REGION_NO_ACCESS or @ref LL_MPU_REGION_PRIV_RW or @ref LL_MPU_REGION_PRIV_RW_URO or @ref LL_MPU_REGION_FULL_ACCESS
  *           or @ref LL_MPU_REGION_PRIV_RO or @ref LL_MPU_REGION_PRIV_RO_URO
  *         @arg @ref LL_MPU_TEX_LEVEL0 or @ref LL_MPU_TEX_LEVEL1 or @ref LL_MPU_TEX_LEVEL2 or @ref LL_MPU_TEX_LEVEL4
  *         @arg @ref LL_MPU_INSTRUCTION_ACCESS_ENABLE or  @ref LL_MPU_INSTRUCTION_ACCESS_DISABLE
  *         @arg @ref LL_MPU_ACCESS_SHAREABLE or @ref LL_MPU_ACCESS_NOT_SHAREABLE
  *         @arg @ref LL_MPU_ACCESS_CACHEABLE or @ref LL_MPU_ACCESS_NOT_CACHEABLE
  *         @arg @ref LL_MPU_ACCESS_BUFFERABLE or @ref LL_MPU_ACCESS_NOT_BUFFERABLE
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_MPU_ConfigRegion(mut Region: uint32_t,
                                         mut SubRegionDisable: uint32_t,
                                         mut Address: uint32_t,
                                         mut Attributes: uint32_t) {
    /* Set Region number */
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd90
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut MPU_Type)).RNR as
                                    *mut uint32_t, Region);
    /* Set base address */
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd90
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut MPU_Type)).RBAR as
                                    *mut uint32_t,
                                Address & 0xffffffe0 as libc::c_uint);
    /* Configure MPU */
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd90
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut MPU_Type)).RASR as
                                    *mut uint32_t,
                                (1 as libc::c_ulong |
                                     Attributes as libc::c_ulong |
                                     (SubRegionDisable << 8 as libc::c_uint)
                                         as libc::c_ulong) as uint32_t);
}
// bootloader/IAP
#[no_mangle]
pub unsafe extern "C" fn systemReset() {
    if mpuResetFn.is_some() {
        mpuResetFn.expect("non-null function pointer")(); // flag that will be readable after reboot
    }
    __disable_irq();
    __NVIC_SystemReset();
}
#[no_mangle]
pub unsafe extern "C" fn systemResetToBootloader() {
    if mpuResetFn.is_some() {
        mpuResetFn.expect("non-null function pointer")();
    }
    ::core::ptr::write_volatile((0x40024000 as
                                     libc::c_uint).wrapping_add(4 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint)
                                    as *mut uint32_t,
                                0xdeadbeef as libc::c_uint);
    __disable_irq();
    __NVIC_SystemReset();
}
#[no_mangle]
pub unsafe extern "C" fn enableGPIOPowerUsageAndNoiseReductions() {
    // AHB1
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         18 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        18 as libc::c_uint);
    let mut tmpreg_0: uint32_t = 0;
    let ref mut fresh1 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         20 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_0 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        20 as libc::c_uint);
    let mut tmpreg_1: uint32_t = 0;
    let ref mut fresh2 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         22 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_1 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        22 as libc::c_uint);
    let mut tmpreg_2: uint32_t = 0;
    let ref mut fresh3 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         29 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_2 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        29 as libc::c_uint);
    let mut tmpreg_3: uint32_t = 0;
    let ref mut fresh4 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         30 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_3 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        30 as libc::c_uint);
    let mut tmpreg_4: uint32_t = 0;
    let ref mut fresh5 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_4 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        0 as libc::c_uint);
    let mut tmpreg_5: uint32_t = 0;
    let ref mut fresh6 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_5 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        1 as libc::c_uint);
    let mut tmpreg_6: uint32_t = 0;
    let ref mut fresh7 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh7,
                                (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_6 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        2 as libc::c_uint);
    let mut tmpreg_7: uint32_t = 0;
    let ref mut fresh8 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh8,
                                (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         3 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_7 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        3 as libc::c_uint);
    let mut tmpreg_8: uint32_t = 0;
    let ref mut fresh9 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh9,
                                (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         4 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_8 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        4 as libc::c_uint);
    let mut tmpreg_9: uint32_t = 0;
    let ref mut fresh10 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         5 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_9 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        5 as libc::c_uint);
    let mut tmpreg_10: uint32_t = 0;
    let ref mut fresh11 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         6 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_10 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        6 as libc::c_uint);
    let mut tmpreg_11: uint32_t = 0;
    let ref mut fresh12 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh12,
                                (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_11 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        7 as libc::c_uint);
    let mut tmpreg_12: uint32_t = 0;
    let ref mut fresh13 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh13,
                                (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_12 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        8 as libc::c_uint);
    //APB1
    let mut tmpreg_13: uint32_t = 0;
    let ref mut fresh14 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh14,
                                (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_13 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        0 as libc::c_uint);
    let mut tmpreg_14: uint32_t = 0;
    let ref mut fresh15 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh15,
                                (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_14 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        1 as libc::c_uint);
    let mut tmpreg_15: uint32_t = 0;
    let ref mut fresh16 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh16,
                                (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_15 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        2 as libc::c_uint);
    let mut tmpreg_16: uint32_t = 0;
    let ref mut fresh17 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh17,
                                (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         3 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_16 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        3 as libc::c_uint);
    let mut tmpreg_17: uint32_t = 0;
    let ref mut fresh18 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh18,
                                (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         4 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_17 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        4 as libc::c_uint);
    let mut tmpreg_18: uint32_t = 0;
    let ref mut fresh19 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh19,
                                (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         5 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_18 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        5 as libc::c_uint);
    let mut tmpreg_19: uint32_t = 0;
    let ref mut fresh20 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh20,
                                (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         6 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_19 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        6 as libc::c_uint);
    let mut tmpreg_20: uint32_t = 0;
    let ref mut fresh21 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh21,
                                (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_20 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        7 as libc::c_uint);
    let mut tmpreg_21: uint32_t = 0;
    let ref mut fresh22 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh22,
                                (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_21 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        8 as libc::c_uint);
    let mut tmpreg_22: uint32_t = 0;
    let ref mut fresh23 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh23,
                                (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         9 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_22 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        9 as libc::c_uint);
    let mut tmpreg_23: uint32_t = 0;
    let ref mut fresh24 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh24,
                                (::core::ptr::read_volatile::<uint32_t>(fresh24
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         14 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_23 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        14 as libc::c_uint);
    let mut tmpreg_24: uint32_t = 0;
    let ref mut fresh25 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh25,
                                (::core::ptr::read_volatile::<uint32_t>(fresh25
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_24 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        15 as libc::c_uint);
    let mut tmpreg_25: uint32_t = 0;
    let ref mut fresh26 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh26,
                                (::core::ptr::read_volatile::<uint32_t>(fresh26
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         17 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_25 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        17 as libc::c_uint);
    let mut tmpreg_26: uint32_t = 0;
    let ref mut fresh27 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh27,
                                (::core::ptr::read_volatile::<uint32_t>(fresh27
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         18 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_26 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        18 as libc::c_uint);
    let mut tmpreg_27: uint32_t = 0;
    let ref mut fresh28 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh28,
                                (::core::ptr::read_volatile::<uint32_t>(fresh28
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         19 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_27 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        19 as libc::c_uint);
    let mut tmpreg_28: uint32_t = 0;
    let ref mut fresh29 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh29,
                                (::core::ptr::read_volatile::<uint32_t>(fresh29
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         20 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_28 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        20 as libc::c_uint);
    let mut tmpreg_29: uint32_t = 0;
    let ref mut fresh30 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh30,
                                (::core::ptr::read_volatile::<uint32_t>(fresh30
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         21 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_29 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        21 as libc::c_uint);
    let mut tmpreg_30: uint32_t = 0;
    let ref mut fresh31 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh31,
                                (::core::ptr::read_volatile::<uint32_t>(fresh31
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         22 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_30 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        22 as libc::c_uint);
    let mut tmpreg_31: uint32_t = 0;
    let ref mut fresh32 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh32,
                                (::core::ptr::read_volatile::<uint32_t>(fresh32
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         23 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_31 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        23 as libc::c_uint);
    let mut tmpreg_32: uint32_t = 0;
    let ref mut fresh33 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh33,
                                (::core::ptr::read_volatile::<uint32_t>(fresh33
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         25 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_32 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        25 as libc::c_uint);
    let mut tmpreg_33: uint32_t = 0;
    let ref mut fresh34 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh34,
                                (::core::ptr::read_volatile::<uint32_t>(fresh34
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         29 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_33 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        29 as libc::c_uint);
    let mut tmpreg_34: uint32_t = 0;
    let ref mut fresh35 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh35,
                                (::core::ptr::read_volatile::<uint32_t>(fresh35
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         30 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_34 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        30 as libc::c_uint);
    let mut tmpreg_35: uint32_t = 0;
    let ref mut fresh36 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh36,
                                (::core::ptr::read_volatile::<uint32_t>(fresh36
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         31 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_35 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        31 as libc::c_uint);
    //APB2
    let mut tmpreg_36: uint32_t = 0;
    let ref mut fresh37 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh37,
                                (::core::ptr::read_volatile::<uint32_t>(fresh37
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_36 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        0 as libc::c_uint);
    let mut tmpreg_37: uint32_t = 0;
    let ref mut fresh38 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh38,
                                (::core::ptr::read_volatile::<uint32_t>(fresh38
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_37 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        1 as libc::c_uint);
    let mut tmpreg_38: uint32_t = 0;
    let ref mut fresh39 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh39,
                                (::core::ptr::read_volatile::<uint32_t>(fresh39
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         4 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_38 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        4 as libc::c_uint);
    let mut tmpreg_39: uint32_t = 0;
    let ref mut fresh40 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh40,
                                (::core::ptr::read_volatile::<uint32_t>(fresh40
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         5 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_39 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        5 as libc::c_uint);
    let mut tmpreg_40: uint32_t = 0;
    let ref mut fresh41 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh41,
                                (::core::ptr::read_volatile::<uint32_t>(fresh41
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_40 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        8 as libc::c_uint);
    let mut tmpreg_41: uint32_t = 0;
    let ref mut fresh42 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh42,
                                (::core::ptr::read_volatile::<uint32_t>(fresh42
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         9 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_41 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        9 as libc::c_uint);
    let mut tmpreg_42: uint32_t = 0;
    let ref mut fresh43 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh43,
                                (::core::ptr::read_volatile::<uint32_t>(fresh43
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         10 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_42 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        10 as libc::c_uint);
    let mut tmpreg_43: uint32_t = 0;
    let ref mut fresh44 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh44,
                                (::core::ptr::read_volatile::<uint32_t>(fresh44
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         11 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_43 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        11 as libc::c_uint);
    let mut tmpreg_44: uint32_t = 0;
    let ref mut fresh45 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh45,
                                (::core::ptr::read_volatile::<uint32_t>(fresh45
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         12 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_44 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        12 as libc::c_uint);
    let mut tmpreg_45: uint32_t = 0;
    let ref mut fresh46 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh46,
                                (::core::ptr::read_volatile::<uint32_t>(fresh46
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         13 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_45 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        13 as libc::c_uint);
    let mut tmpreg_46: uint32_t = 0;
    let ref mut fresh47 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh47,
                                (::core::ptr::read_volatile::<uint32_t>(fresh47
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         16 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_46 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        16 as libc::c_uint);
    let mut tmpreg_47: uint32_t = 0;
    let ref mut fresh48 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh48,
                                (::core::ptr::read_volatile::<uint32_t>(fresh48
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         17 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_47 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        17 as libc::c_uint);
    let mut tmpreg_48: uint32_t = 0;
    let ref mut fresh49 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh49,
                                (::core::ptr::read_volatile::<uint32_t>(fresh49
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         18 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_48 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        18 as libc::c_uint);
    let mut tmpreg_49: uint32_t = 0;
    let ref mut fresh50 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh50,
                                (::core::ptr::read_volatile::<uint32_t>(fresh50
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         20 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_49 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        20 as libc::c_uint);
    let mut tmpreg_50: uint32_t = 0;
    let ref mut fresh51 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh51,
                                (::core::ptr::read_volatile::<uint32_t>(fresh51
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         22 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_50 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        22 as libc::c_uint);
    let mut tmpreg_51: uint32_t = 0;
    let ref mut fresh52 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh52,
                                (::core::ptr::read_volatile::<uint32_t>(fresh52
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         23 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_51 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB2ENR &
                                    (0x1 as libc::c_uint) <<
                                        23 as libc::c_uint);
    //
//    GPIO_InitTypeDef GPIO_InitStructure;
//    GPIO_StructInit(&GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // default is un-pulled input
//
//    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
//    GPIO_InitStructure.GPIO_Pin &= ~(GPIO_Pin_11 | GPIO_Pin_12); // leave USB D+/D- alone
//
//    GPIO_InitStructure.GPIO_Pin &= ~(GPIO_Pin_13 | GPIO_Pin_14); // leave JTAG pins alone
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//    GPIO_Init(GPIOD, &GPIO_InitStructure);
//    GPIO_Init(GPIOE, &GPIO_InitStructure);
}
#[no_mangle]
pub unsafe extern "C" fn isMPUSoftReset() -> bool {
    if cachedRccCsrValue & (0x1 as libc::c_uint) << 28 as libc::c_uint != 0 {
        return 1 as libc::c_int != 0
    } else { return 0 as libc::c_int != 0 };
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#[no_mangle]
pub unsafe extern "C" fn systemInit() {
    checkForBootLoaderRequest();
    //  Mark ITCM-RAM as read-only
    LL_MPU_ConfigRegion(0 as libc::c_uint, 0 as libc::c_int as uint32_t,
                        0 as libc::c_uint,
                        (0xd as libc::c_uint) << 1 as libc::c_uint |
                            (0x6 as libc::c_uint) << 24 as libc::c_uint);
    LL_MPU_Enable(((1 as libc::c_ulong) << 2 as libc::c_uint) as uint32_t);
    //SystemClock_Config();
    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(0x5 as libc::c_uint);
    // cache RCC->CSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CSR;
    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
    //extern void *isr_vector_table_base;
    //NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
    //__HAL_RCC_USB_OTG_FS_CLK_DISABLE;
    //RCC_ClearFlag();
    enableGPIOPowerUsageAndNoiseReductions();
    // Init cycle counter
    cycleCounterInit();
    // SysTick
    //SysTick_Config(SystemCoreClock / 1000);
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq().wrapping_div(1000 as libc::c_int
                                                              as
                                                              libc::c_uint)); // Reset our trigger
    HAL_SYSTICK_CLKSourceConfig(0x4 as libc::c_uint);
}
#[no_mangle]
pub static mut bootJump: Option<unsafe extern "C" fn() -> ()> = None;
#[no_mangle]
pub unsafe extern "C" fn checkForBootLoaderRequest() {
    let mut bt: uint32_t = 0;
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh53 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh53,
                                (::core::ptr::read_volatile::<uint32_t>(fresh53
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         28 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        28 as libc::c_uint);
    let mut tmpreg_0: uint32_t = 0;
    let ref mut fresh54 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh54,
                                (::core::ptr::read_volatile::<uint32_t>(fresh54
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         18 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_0 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        18 as libc::c_uint);
    HAL_PWR_EnableBkUpAccess();
    bt =
        *((0x40024000 as
               libc::c_uint).wrapping_add(4 as libc::c_int as libc::c_uint) as
              *mut uint32_t);
    if bt == 0xdeadbeef as libc::c_uint {
        ::core::ptr::write_volatile((0x40024000 as
                                         libc::c_uint).wrapping_add(4 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                        as *mut uint32_t,
                                    0xcafefeed as libc::c_uint);
        // Backup SRAM is write-back by default, ensure value actually reaches memory
        // Another solution would be marking BKPSRAM as write-through in Memory Protection Unit settings
        SCB_CleanDCache_by_Addr((0x40024000 as
                                     libc::c_uint).wrapping_add(4 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint)
                                    as *mut uint32_t,
                                ::core::mem::size_of::<uint32_t>() as
                                    libc::c_ulong as
                                    int32_t); //Set the main stack pointer to its defualt values
        let mut SysMemBootJump: Option<unsafe extern "C" fn() -> ()> =
            None; // Point the PC to the System Memory reset vector (+4)
        let mut tmpreg_1: uint32_t = 0;
        let ref mut fresh55 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).APB2ENR;
        ::core::ptr::write_volatile(fresh55,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh55
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             14 as libc::c_uint) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_1 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).APB2ENR &
                                        (0x1 as libc::c_uint) <<
                                            14 as libc::c_uint);
        let ref mut fresh56 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut SYSCFG_TypeDef)).MEMRMP;
        ::core::ptr::write_volatile(fresh56,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh56
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0 as libc::c_uint)
                                        as uint32_t as uint32_t);
        let mut p: uint32_t = *(0x1ff00000 as libc::c_int as *mut uint32_t);
        __set_MSP(p);
        SysMemBootJump =
            ::core::mem::transmute::<libc::intptr_t,
                                     Option<unsafe extern "C" fn()
                                                ->
                                                    ()>>(*(0x1ff00004 as
                                                               libc::c_int as
                                                               *mut uint32_t)
                                                             as
                                                             libc::intptr_t);
        SysMemBootJump.expect("non-null function pointer")();
        loop  { }
    };
}
