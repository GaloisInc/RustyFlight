use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_GetClocksFreq(RCC_Clocks: *mut RCC_ClocksTypeDef);
    #[no_mangle]
    fn RCC_APB2PeriphResetCmd(RCC_APB2Periph: uint32_t,
                              NewState: FunctionalState);
    #[no_mangle]
    fn RCC_APB1PeriphResetCmd(RCC_APB1Periph: uint32_t,
                              NewState: FunctionalState);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
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
  * @brief Serial Peripheral Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR: uint16_t,
    pub RESERVED3: uint16_t,
    pub CRCPR: uint16_t,
    pub RESERVED4: uint16_t,
    pub RXCRCR: uint16_t,
    pub RESERVED5: uint16_t,
    pub TXCRCR: uint16_t,
    pub RESERVED6: uint16_t,
    pub I2SCFGR: uint16_t,
    pub RESERVED7: uint16_t,
    pub I2SPR: uint16_t,
    pub RESERVED8: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f30x_rcc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the RCC 
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
/* * @addtogroup RCC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_ClocksTypeDef {
    pub SYSCLK_Frequency: uint32_t,
    pub HCLK_Frequency: uint32_t,
    pub PCLK1_Frequency: uint32_t,
    pub PCLK2_Frequency: uint32_t,
    pub ADC12CLK_Frequency: uint32_t,
    pub ADC34CLK_Frequency: uint32_t,
    pub I2C1CLK_Frequency: uint32_t,
    pub I2C2CLK_Frequency: uint32_t,
    pub I2C3CLK_Frequency: uint32_t,
    pub TIM1CLK_Frequency: uint32_t,
    pub HRTIM1CLK_Frequency: uint32_t,
    pub TIM8CLK_Frequency: uint32_t,
    pub USART1CLK_Frequency: uint32_t,
    pub USART2CLK_Frequency: uint32_t,
    pub USART3CLK_Frequency: uint32_t,
    pub UART4CLK_Frequency: uint32_t,
    pub UART5CLK_Frequency: uint32_t,
    pub TIM15CLK_Frequency: uint32_t,
    pub TIM16CLK_Frequency: uint32_t,
    pub TIM17CLK_Frequency: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_InitTypeDef {
    pub SPI_Direction: uint16_t,
    pub SPI_Mode: uint16_t,
    pub SPI_DataSize: uint16_t,
    pub SPI_CPOL: uint16_t,
    pub SPI_CPHA: uint16_t,
    pub SPI_NSS: uint16_t,
    pub SPI_BaudRatePrescaler: uint16_t,
    pub SPI_FirstBit: uint16_t,
    pub SPI_CRCPolynomial: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2S_InitTypeDef {
    pub I2S_Mode: uint16_t,
    pub I2S_Standard: uint16_t,
    pub I2S_DataFormat: uint16_t,
    pub I2S_MCLKOutput: uint16_t,
    pub I2S_AudioFreq: uint32_t,
    pub I2S_CPOL: uint16_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup SPI_Private_Functions
  * @{
  */
/* * @defgroup SPI_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions 
 *
@verbatim   
 ===============================================================================
           ##### Initialization and Configuration functions #####
 ===============================================================================  
    [..] This section provides a set of functions allowing to initialize the SPI Direction,
         SPI Mode, SPI Data Size, SPI Polarity, SPI Phase, SPI NSS Management, SPI Baud
         Rate Prescaler, SPI First Bit and SPI CRC Polynomial.
    [..] The SPI_Init() function follows the SPI configuration procedures for Master mode
         and Slave mode (details for these procedures are available in reference manual).
    [..] When the Software NSS management (SPI_InitStruct->SPI_NSS = SPI_NSS_Soft) is selected,
         use the following function to manage the NSS bit:
         void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
    [..] In Master mode, when the Hardware NSS management (SPI_InitStruct->SPI_NSS = SPI_NSS_Hard)
         is selected, use the follwoing function to enable the NSS output feature.
         void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
    [..] The NSS pulse mode can be managed by the SPI TI mode when enabling it using the 
         following function: void SPI_TIModeCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
         And it can be managed by software in the SPI Motorola mode using this function: 
         void SPI_NSSPulseModeCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
    [..] This section provides also functions to initialize the I2S Mode, Standard, 
         Data Format, MCLK Output, Audio frequency and Polarity.
    [..] The I2S_Init() function follows the I2S configuration procedures for Master mode
         and Slave mode.
  
@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the SPIx peripheral registers to their default
  *         reset values.
  * @param  SPIx: To select the SPIx peripheral, where x can be: 1, 2 or 3 
  *         in SPI mode.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_DeInit(mut SPIx: *mut SPI_TypeDef) {
    /* Check the parameters */
    if SPIx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x3000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut SPI_TypeDef {
        /* Enable SPI1 reset state */
        RCC_APB2PeriphResetCmd(0x1000 as libc::c_int as uint32_t, ENABLE);
        /* Release SPI1 from reset state */
        RCC_APB2PeriphResetCmd(0x1000 as libc::c_int as uint32_t, DISABLE);
    } else if SPIx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x3800 as libc::c_int as
                                                  libc::c_uint) as
                      *mut SPI_TypeDef {
        /* Enable SPI2 reset state */
        RCC_APB1PeriphResetCmd(0x4000 as libc::c_int as uint32_t, ENABLE);
        /* Release SPI2 from reset state */
        RCC_APB1PeriphResetCmd(0x4000 as libc::c_int as uint32_t, DISABLE);
    } else if SPIx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x3c00 as libc::c_int as
                                                  libc::c_uint) as
                      *mut SPI_TypeDef {
        /* Enable SPI3 reset state */
        RCC_APB1PeriphResetCmd(0x8000 as libc::c_int as uint32_t, ENABLE);
        /* Release SPI3 from reset state */
        RCC_APB1PeriphResetCmd(0x8000 as libc::c_int as uint32_t, DISABLE);
    };
}
/* *
  * @brief  Fills each SPI_InitStruct member with its default value.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_StructInit(mut SPI_InitStruct:
                                            *mut SPI_InitTypeDef) {
    /*--------------- Reset SPI init structure parameters values -----------------*/
  /* Initialize the SPI_Direction member */
    (*SPI_InitStruct).SPI_Direction = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_Mode member */
    (*SPI_InitStruct).SPI_Mode = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_DataSize member */
    (*SPI_InitStruct).SPI_DataSize = 0x700 as libc::c_int as uint16_t;
    /* Initialize the SPI_CPOL member */
    (*SPI_InitStruct).SPI_CPOL = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_CPHA member */
    (*SPI_InitStruct).SPI_CPHA = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_NSS member */
    (*SPI_InitStruct).SPI_NSS = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_BaudRatePrescaler member */
    (*SPI_InitStruct).SPI_BaudRatePrescaler = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_FirstBit member */
    (*SPI_InitStruct).SPI_FirstBit = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_CRCPolynomial member */
    (*SPI_InitStruct).SPI_CRCPolynomial = 7 as libc::c_int as uint16_t;
}
/* *
  * @brief  Initializes the SPIx peripheral according to the specified 
  *         parameters in the SPI_InitStruct.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure that
  *         contains the configuration information for the specified SPI peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_Init(mut SPIx: *mut SPI_TypeDef,
                                  mut SPI_InitStruct: *mut SPI_InitTypeDef) {
    let mut tmpreg: uint16_t = 0 as libc::c_int as uint16_t;
    /* check the parameters */
    /* Configuring the SPI in master mode */
    if (*SPI_InitStruct).SPI_Mode as libc::c_int ==
           0x104 as libc::c_int as uint16_t as libc::c_int {
        /*---------------------------- SPIx CR1 Configuration ------------------------*/
    /* Get the SPIx CR1 value */
        tmpreg = (*SPIx).CR1;
        /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, MSTR, CPOL and CPHA bits */
        tmpreg =
            (tmpreg as libc::c_int &
                 0x3040 as libc::c_int as uint16_t as libc::c_int) as
                uint16_t;
        /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
       master/slave mode, CPOL and CPHA */
    /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
    /* Set SSM, SSI and MSTR bits according to SPI_Mode and SPI_NSS values */
    /* Set LSBFirst bit according to SPI_FirstBit value */
    /* Set BR bits according to SPI_BaudRatePrescaler value */
    /* Set CPOL bit according to SPI_CPOL value */
    /* Set CPHA bit according to SPI_CPHA value */
        tmpreg =
            (tmpreg as libc::c_int |
                 (((*SPI_InitStruct).SPI_Direction as libc::c_int |
                       (*SPI_InitStruct).SPI_Mode as libc::c_int) as uint16_t
                      as libc::c_int |
                      (((*SPI_InitStruct).SPI_CPOL as libc::c_int |
                            (*SPI_InitStruct).SPI_CPHA as libc::c_int) as
                           uint16_t as libc::c_int |
                           (((*SPI_InitStruct).SPI_NSS as libc::c_int |
                                 (*SPI_InitStruct).SPI_BaudRatePrescaler as
                                     libc::c_int) as uint16_t as libc::c_int |
                                (*SPI_InitStruct).SPI_FirstBit as libc::c_int)
                               as uint16_t as libc::c_int) as uint16_t as
                          libc::c_int) as uint16_t as libc::c_int) as
                uint16_t;
        /* Write to SPIx CR1 */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    tmpreg);
        /*-------------------------Data Size Configuration -----------------------*/
    /* Get the SPIx CR2 value */
        tmpreg = (*SPIx).CR2;
        /* Clear DS[3:0] bits */
        tmpreg =
            (tmpreg as libc::c_int &
                 !(0xf00 as libc::c_int as uint16_t as libc::c_int) as
                     uint16_t as libc::c_int) as uint16_t;
        /* Configure SPIx: Data Size */
        tmpreg =
            (tmpreg as libc::c_int |
                 (*SPI_InitStruct).SPI_DataSize as libc::c_int) as uint16_t;
        /* Write to SPIx CR2 */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t, tmpreg)
    } else {
        /* Configuring the SPI in slave mode */
        /*---------------------------- Data size Configuration -----------------------*/
    /* Get the SPIx CR2 value */
        tmpreg = (*SPIx).CR2;
        tmpreg =
            (tmpreg as libc::c_int &
                 !(0xf00 as libc::c_int as uint16_t as libc::c_int) as
                     uint16_t as libc::c_int) as uint16_t;
        tmpreg =
            (tmpreg as libc::c_int |
                 (*SPI_InitStruct).SPI_DataSize as libc::c_int) as uint16_t;
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    tmpreg);
        tmpreg = (*SPIx).CR1;
        tmpreg =
            (tmpreg as libc::c_int &
                 0x3040 as libc::c_int as uint16_t as libc::c_int) as
                uint16_t;
        tmpreg =
            (tmpreg as libc::c_int |
                 (((*SPI_InitStruct).SPI_Direction as libc::c_int |
                       (*SPI_InitStruct).SPI_Mode as libc::c_int) as uint16_t
                      as libc::c_int |
                      (((*SPI_InitStruct).SPI_CPOL as libc::c_int |
                            (*SPI_InitStruct).SPI_CPHA as libc::c_int) as
                           uint16_t as libc::c_int |
                           (((*SPI_InitStruct).SPI_NSS as libc::c_int |
                                 (*SPI_InitStruct).SPI_BaudRatePrescaler as
                                     libc::c_int) as uint16_t as libc::c_int |
                                (*SPI_InitStruct).SPI_FirstBit as libc::c_int)
                               as uint16_t as libc::c_int) as uint16_t as
                          libc::c_int) as uint16_t as libc::c_int) as
                uint16_t;
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t, tmpreg)
    }
    /* Clear DS[3:0] bits */
    /* Configure SPIx: Data Size */
    /* Write to SPIx CR2 */
    /*---------------------------- SPIx CR1 Configuration ------------------------*/
    /* Get the SPIx CR1 value */
    /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, MSTR, CPOL and CPHA bits */
    /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
       master/salve mode, CPOL and CPHA */
    /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
    /* Set SSM, SSI and MSTR bits according to SPI_Mode and SPI_NSS values */
    /* Set LSBFirst bit according to SPI_FirstBit value */
    /* Set BR bits according to SPI_BaudRatePrescaler value */
    /* Set CPOL bit according to SPI_CPOL value */
    /* Set CPHA bit according to SPI_CPHA value */
    /* Write to SPIx CR1 */
    /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
    ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).I2SCFGR
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(0x800 as libc::c_int as uint16_t as
                                           libc::c_int) as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /*---------------------------- SPIx CRCPOLY Configuration --------------------*/
  /* Write to SPIx CRCPOLY */
    ::core::ptr::write_volatile(&mut (*SPIx).CRCPR as *mut uint16_t,
                                (*SPI_InitStruct).SPI_CRCPolynomial);
}
/* *
  * @brief  Fills each I2S_InitStruct member with its default value.
  * @param  I2S_InitStruct : pointer to a I2S_InitTypeDef structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2S_StructInit(mut I2S_InitStruct:
                                            *mut I2S_InitTypeDef) {
    /*--------------- Reset I2S init structure parameters values -----------------*/
  /* Initialize the I2S_Mode member */
    (*I2S_InitStruct).I2S_Mode = 0 as libc::c_int as uint16_t;
    /* Initialize the I2S_Standard member */
    (*I2S_InitStruct).I2S_Standard = 0 as libc::c_int as uint16_t;
    /* Initialize the I2S_DataFormat member */
    (*I2S_InitStruct).I2S_DataFormat = 0 as libc::c_int as uint16_t;
    /* Initialize the I2S_MCLKOutput member */
    (*I2S_InitStruct).I2S_MCLKOutput = 0 as libc::c_int as uint16_t;
    /* Initialize the I2S_AudioFreq member */
    (*I2S_InitStruct).I2S_AudioFreq = 2 as libc::c_int as uint32_t;
    /* Initialize the I2S_CPOL member */
    (*I2S_InitStruct).I2S_CPOL = 0 as libc::c_int as uint16_t;
}
/* *
  * @brief  Initializes the SPIx peripheral according to the specified 
  *   parameters in the I2S_InitStruct.
  * @param  SPIx:To select the SPIx peripheral, where x can be: 2 or 3 
  *         in I2S mode. 
  * @param  I2S_InitStruct: pointer to an I2S_InitTypeDef structure that
  *   contains the configuration information for the specified SPI peripheral
  *   configured in I2S mode.
  * @note
  *  The function calculates the optimal prescaler needed to obtain the most 
  *  accurate audio frequency (depending on the I2S clock source, the PLL values 
  *  and the product configuration). But in case the prescaler value is greater 
  *  than 511, the default value (0x02) will be configured instead.     
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2S_Init(mut SPIx: *mut SPI_TypeDef,
                                  mut I2S_InitStruct: *mut I2S_InitTypeDef) {
    let mut tmpreg: uint16_t = 0 as libc::c_int as uint16_t;
    let mut i2sdiv: uint16_t = 2 as libc::c_int as uint16_t;
    let mut i2sodd: uint16_t = 0 as libc::c_int as uint16_t;
    let mut packetlength: uint16_t = 1 as libc::c_int as uint16_t;
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut RCC_Clocks: RCC_ClocksTypeDef =
        RCC_ClocksTypeDef{SYSCLK_Frequency: 0,
                          HCLK_Frequency: 0,
                          PCLK1_Frequency: 0,
                          PCLK2_Frequency: 0,
                          ADC12CLK_Frequency: 0,
                          ADC34CLK_Frequency: 0,
                          I2C1CLK_Frequency: 0,
                          I2C2CLK_Frequency: 0,
                          I2C3CLK_Frequency: 0,
                          TIM1CLK_Frequency: 0,
                          HRTIM1CLK_Frequency: 0,
                          TIM8CLK_Frequency: 0,
                          USART1CLK_Frequency: 0,
                          USART2CLK_Frequency: 0,
                          USART3CLK_Frequency: 0,
                          UART4CLK_Frequency: 0,
                          UART5CLK_Frequency: 0,
                          TIM15CLK_Frequency: 0,
                          TIM16CLK_Frequency: 0,
                          TIM17CLK_Frequency: 0,};
    let mut sourceclock: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the I2S parameters */
    /*----------------------- SPIx I2SCFGR & I2SPR Configuration -----------------*/
  /* Clear I2SMOD, I2SE, I2SCFG, PCMSYNC, I2SSTD, CKPOL, DATLEN and CHLEN bits */
    ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).I2SCFGR
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xf040 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    ::core::ptr::write_volatile(&mut (*SPIx).I2SPR as *mut uint16_t,
                                0x2 as libc::c_int as uint16_t);
    /* Get the I2SCFGR register value */
    tmpreg = (*SPIx).I2SCFGR;
    /* If the default value has to be written, reinitialize i2sdiv and i2sodd*/
    if (*I2S_InitStruct).I2S_AudioFreq == 2 as libc::c_int as uint32_t {
        i2sodd = 0 as libc::c_int as uint16_t;
        i2sdiv = 2 as libc::c_int as uint16_t
    } else {
        /* If the requested audio frequency is not the default, compute the prescaler */
        /* Check the frame length (For the Prescaler computing) */
        if (*I2S_InitStruct).I2S_DataFormat as libc::c_int ==
               0 as libc::c_int as uint16_t as libc::c_int {
            /* Packet length is 16 bits */
            packetlength = 1 as libc::c_int as uint16_t
        } else {
            /* Packet length is 32 bits */
            packetlength = 2 as libc::c_int as uint16_t
        }
        RCC_GetClocksFreq(&mut RCC_Clocks);
        sourceclock = RCC_Clocks.SYSCLK_Frequency;
        if (*I2S_InitStruct).I2S_MCLKOutput as libc::c_int ==
               0x200 as libc::c_int as uint16_t as libc::c_int {
            /* I2S Clock source is System clock: Get System Clock frequency */
            /* Get the source clock value: based on System Clock value */
            /* MCLK output is enabled */
            tmp =
                sourceclock.wrapping_div(256 as libc::c_int as
                                             libc::c_uint).wrapping_mul(10 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_div((*I2S_InitStruct).I2S_AudioFreq).wrapping_add(5
                                                                                                                                                         as
                                                                                                                                                         libc::c_int
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                    as uint16_t as uint32_t
        } else {
            /* MCLK output is disabled */
            tmp =
                sourceclock.wrapping_div((32 as libc::c_int *
                                              packetlength as libc::c_int) as
                                             libc::c_uint).wrapping_mul(10 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_div((*I2S_InitStruct).I2S_AudioFreq).wrapping_add(5
                                                                                                                                                         as
                                                                                                                                                         libc::c_int
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                    as uint16_t as uint32_t
        }
        tmp = tmp.wrapping_div(10 as libc::c_int as libc::c_uint);
        i2sodd =
            (tmp & 0x1 as libc::c_int as uint16_t as libc::c_uint) as
                uint16_t;
        i2sdiv =
            tmp.wrapping_sub(i2sodd as
                                 libc::c_uint).wrapping_div(2 as libc::c_int
                                                                as
                                                                libc::c_uint)
                as uint16_t;
        i2sodd = ((i2sodd as libc::c_int) << 8 as libc::c_int) as uint16_t
    }
    /* Compute the Real divider depending on the MCLK output state with a floating point */
    /* Remove the floating point */
    /* Check the parity of the divider */
    /* Compute the i2sdiv prescaler */
    /* Get the Mask for the Odd bit (SPI_I2SPR[8]) register */
    /* Test if the divider is 1 or 0 or greater than 0xFF */
    if (i2sdiv as libc::c_int) < 2 as libc::c_int ||
           i2sdiv as libc::c_int > 0xff as libc::c_int {
        /* Set the default values */
        i2sdiv = 2 as libc::c_int as uint16_t;
        i2sodd = 0 as libc::c_int as uint16_t
    }
    /* Write to SPIx I2SPR register the computed value */
    ::core::ptr::write_volatile(&mut (*SPIx).I2SPR as *mut uint16_t,
                                (i2sdiv as libc::c_int |
                                     (i2sodd as libc::c_int |
                                          (*I2S_InitStruct).I2S_MCLKOutput as
                                              libc::c_int) as uint16_t as
                                         libc::c_int) as uint16_t);
    /* Configure the I2S with the SPI_InitStruct values */
    tmpreg =
        (tmpreg as libc::c_int |
             ((0x800 as libc::c_int as uint16_t as libc::c_int |
                   (*I2S_InitStruct).I2S_Mode as libc::c_int) as uint16_t as
                  libc::c_int |
                  (((*I2S_InitStruct).I2S_Standard as libc::c_int |
                        (*I2S_InitStruct).I2S_DataFormat as libc::c_int) as
                       uint16_t as libc::c_int |
                       (*I2S_InitStruct).I2S_CPOL as libc::c_int) as uint16_t
                      as libc::c_int) as uint16_t as libc::c_int) as uint16_t;
    /* Write to SPIx I2SCFGR */
    ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint16_t,
                                tmpreg);
}
/* *
  * @brief  Enables or disables the specified SPI peripheral.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx peripheral. 
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_Cmd(mut SPIx: *mut SPI_TypeDef,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI peripheral */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x40 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected SPI peripheral */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x40 as libc::c_int as uint16_t as
                                               libc::c_int) as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the TI Mode.
  * @note    This function can be called only after the SPI_Init() function has 
  *          been called. 
  * @note    When TI mode is selected, the control bits SSM, SSI, CPOL and CPHA 
  *          are not taken into consideration and are configured by hardware 
  *          respectively to the TI mode requirements.  
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.  
  * @param  NewState: new state of the selected SPI TI communication mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_TIModeCmd(mut SPIx: *mut SPI_TypeDef,
                                       mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the TI mode for the selected SPI peripheral */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x10 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the TI mode for the selected SPI peripheral */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x10 as libc::c_int as uint16_t as
                                               libc::c_int) as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the specified SPI peripheral (in I2S mode).
  * @param  SPIx:To select the SPIx peripheral, where x can be: 2 or 3 in 
  *         I2S mode or I2Sxext for I2S full duplex mode. 
  * @param  NewState: new state of the SPIx peripheral. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2S_Cmd(mut SPIx: *mut SPI_TypeDef,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI peripheral in I2S mode */
        ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).I2SCFGR
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x400 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected SPI peripheral in I2S mode */
        ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).I2SCFGR
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x400 as libc::c_int as uint16_t as
                                               libc::c_int) as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Configures the data size for the selected SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral. 
  * @param  SPI_DataSize: specifies the SPI data size.
  *   For the SPIx peripheral this parameter can be one of the following values:
  *     @arg SPI_DataSize_4b: Set data size to 4 bits
  *     @arg SPI_DataSize_5b: Set data size to 5 bits
  *     @arg SPI_DataSize_6b: Set data size to 6 bits
  *     @arg SPI_DataSize_7b: Set data size to 7 bits
  *     @arg SPI_DataSize_8b: Set data size to 8 bits
  *     @arg SPI_DataSize_9b: Set data size to 9 bits
  *     @arg SPI_DataSize_10b: Set data size to 10 bits
  *     @arg SPI_DataSize_11b: Set data size to 11 bits
  *     @arg SPI_DataSize_12b: Set data size to 12 bits
  *     @arg SPI_DataSize_13b: Set data size to 13 bits
  *     @arg SPI_DataSize_14b: Set data size to 14 bits
  *     @arg SPI_DataSize_15b: Set data size to 15 bits
  *     @arg SPI_DataSize_16b: Set data size to 16 bits
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_DataSizeConfig(mut SPIx: *mut SPI_TypeDef,
                                            mut SPI_DataSize: uint16_t) {
    let mut tmpreg: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    /* Read the CR2 register */
    tmpreg = (*SPIx).CR2;
    /* Clear DS[3:0] bits */
    tmpreg =
        (tmpreg as libc::c_int &
             !(0xf00 as libc::c_int as uint16_t as libc::c_int) as uint16_t as
                 libc::c_int) as uint16_t;
    /* Set new DS[3:0] bits value */
    tmpreg =
        (tmpreg as libc::c_int | SPI_DataSize as libc::c_int) as uint16_t;
    ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t, tmpreg);
}
/* *
  * @brief  Configures the FIFO reception threshold for the selected SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral. 
  * @param  SPI_RxFIFOThreshold: specifies the FIFO reception threshold.
  *   This parameter can be one of the following values:
  *     @arg SPI_RxFIFOThreshold_HF: RXNE event is generated if the FIFO 
  *          level is greater or equal to 1/2. 
  *     @arg SPI_RxFIFOThreshold_QF: RXNE event is generated if the FIFO 
  *          level is greater or equal to 1/4. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_RxFIFOThresholdConfig(mut SPIx: *mut SPI_TypeDef,
                                                   mut SPI_RxFIFOThreshold:
                                                       uint16_t) {
    /* Check the parameters */
    /* Clear FRXTH bit */
    ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(0x1000 as libc::c_int as uint16_t as
                                           libc::c_int) as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /* Set new FRXTH bit value */
    ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     SPI_RxFIFOThreshold as libc::c_int) as
                                    uint16_t as uint16_t);
}
/* *
  * @brief  Selects the data transfer direction in bidirectional mode for the specified SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral. 
  * @param  SPI_Direction: specifies the data transfer direction in bidirectional mode. 
  *   This parameter can be one of the following values:
  *     @arg SPI_Direction_Tx: Selects Tx transmission direction
  *     @arg SPI_Direction_Rx: Selects Rx receive direction
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_BiDirectionalLineConfig(mut SPIx:
                                                         *mut SPI_TypeDef,
                                                     mut SPI_Direction:
                                                         uint16_t) {
    /* Check the parameters */
    if SPI_Direction as libc::c_int ==
           0x4000 as libc::c_int as uint16_t as libc::c_int {
        /* Set the Tx only mode */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x4000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Set the Rx only mode */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xbfff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Configures internally by software the NSS pin for the selected SPI.
  * @note    This function can be called only after the SPI_Init() function has 
  *          been called.  
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_NSSInternalSoft: specifies the SPI NSS internal state.
  *   This parameter can be one of the following values:
  *     @arg SPI_NSSInternalSoft_Set: Set NSS pin internally
  *     @arg SPI_NSSInternalSoft_Reset: Reset NSS pin internally
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_NSSInternalSoftwareConfig(mut SPIx:
                                                           *mut SPI_TypeDef,
                                                       mut SPI_NSSInternalSoft:
                                                           uint16_t) {
    /* Check the parameters */
    if SPI_NSSInternalSoft as libc::c_int !=
           0xfeff as libc::c_int as uint16_t as libc::c_int {
        /* Set NSS pin internally by software */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x100 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Reset NSS pin internally by software */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfeff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Configures the full duplex mode for the I2Sx peripheral using its
  *         extension I2Sxext according to the specified parameters in the 
  *         I2S_InitStruct.
  * @param  I2Sxext: where x can be  2 or 3 to select the I2S peripheral extension block.
  * @param  I2S_InitStruct: pointer to an I2S_InitTypeDef structure that
  *         contains the configuration information for the specified I2S peripheral
  *         extension.
  * 
  * @note   The structure pointed by I2S_InitStruct parameter should be the same
  *         used for the master I2S peripheral. In this case, if the master is 
  *         configured as transmitter, the slave will be receiver and vice versa.
  *         Or you can force a different mode by modifying the field I2S_Mode to the
  *         value I2S_SlaveRx or I2S_SlaveTx indepedently of the master configuration.    
  *         
  * @note   The I2S full duplex extension can be configured in slave mode only.    
  *  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2S_FullDuplexConfig(mut I2Sxext: *mut SPI_TypeDef,
                                              mut I2S_InitStruct:
                                                  *mut I2S_InitTypeDef) {
    let mut tmpreg: uint16_t = 0 as libc::c_int as uint16_t;
    let mut tmp: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the I2S parameters */
    /*----------------------- SPIx I2SCFGR & I2SPR Configuration -----------------*/
  /* Clear I2SMOD, I2SE, I2SCFG, PCMSYNC, I2SSTD, CKPOL, DATLEN and CHLEN bits */
    ::core::ptr::write_volatile(&mut (*I2Sxext).I2SCFGR as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*I2Sxext).I2SCFGR
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xf040 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    ::core::ptr::write_volatile(&mut (*I2Sxext).I2SPR as *mut uint16_t,
                                0x2 as libc::c_int as uint16_t);
    /* Get the I2SCFGR register value */
    tmpreg = (*I2Sxext).I2SCFGR;
    /* Get the mode to be configured for the extended I2S */
    if (*I2S_InitStruct).I2S_Mode as libc::c_int ==
           0x200 as libc::c_int as uint16_t as libc::c_int ||
           (*I2S_InitStruct).I2S_Mode as libc::c_int ==
               0 as libc::c_int as uint16_t as libc::c_int {
        tmp = 0x100 as libc::c_int as uint16_t
    } else if (*I2S_InitStruct).I2S_Mode as libc::c_int ==
                  0x300 as libc::c_int as uint16_t as libc::c_int ||
                  (*I2S_InitStruct).I2S_Mode as libc::c_int ==
                      0x100 as libc::c_int as uint16_t as libc::c_int {
        tmp = 0 as libc::c_int as uint16_t
    }
    /* Configure the I2S with the SPI_InitStruct values */
    tmpreg =
        (tmpreg as libc::c_int |
             (0x800 as libc::c_int as uint16_t as libc::c_int |
                  (tmp as libc::c_int |
                       ((*I2S_InitStruct).I2S_Standard as libc::c_int |
                            ((*I2S_InitStruct).I2S_DataFormat as libc::c_int |
                                 (*I2S_InitStruct).I2S_CPOL as libc::c_int) as
                                uint16_t as libc::c_int) as uint16_t as
                           libc::c_int) as uint16_t as libc::c_int) as
                 uint16_t as libc::c_int) as uint16_t;
    /* Write to SPIx I2SCFGR */
    ::core::ptr::write_volatile(&mut (*I2Sxext).I2SCFGR as *mut uint16_t,
                                tmpreg);
}
/* *
  * @brief  Enables or disables the SS output for the selected SPI.
  * @note    This function can be called only after the SPI_Init() function has 
  *          been called and the NSS hardware management mode is selected. 
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx SS output. 
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_SSOutputCmd(mut SPIx: *mut SPI_TypeDef,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI SS output */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x4 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected SPI SS output */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x4 as libc::c_int as uint16_t as
                                               libc::c_int) as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the NSS pulse management mode.
  * @note    This function can be called only after the SPI_Init() function has 
  *          been called. 
  * @note    When TI mode is selected, the control bits NSSP is not taken into 
  *          consideration and are configured by hardware respectively to the 
  *          TI mode requirements. 
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral. 
  * @param  NewState: new state of the NSS pulse management mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_NSSPulseModeCmd(mut SPIx: *mut SPI_TypeDef,
                                             mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the NSS pulse management mode */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x8 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the NSS pulse management mode */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x8 as libc::c_int as uint16_t as
                                               libc::c_int) as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @}
  */
/* * @defgroup SPI_Group2 Data transfers functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                    ##### Data transfers functions #####
 ===============================================================================  
    [..] This section provides a set of functions allowing to manage the SPI or I2S 
         data transfers.
    [..] In reception, data are received and then stored into an internal Rx buffer while 
         In transmission, data are first stored into an internal Tx buffer before being 
         transmitted.
    [..] The read access of the SPI_DR register can be done using the SPI_I2S_ReceiveData()
         function and returns the Rx buffered value. Whereas a write access to the SPI_DR 
         can be done using SPI_I2S_SendData() function and stores the written data into 
         Tx buffer.

@endverbatim
  * @{
  */
/* *
  * @brief  Transmits a Data through the SPIx peripheral.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  Data: Data to be transmitted.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_SendData8(mut SPIx: *mut SPI_TypeDef,
                                       mut Data: uint8_t) {
    let mut spixbase: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    spixbase = SPIx as uint32_t;
    spixbase =
        (spixbase as
             libc::c_uint).wrapping_add(0xc as libc::c_int as libc::c_uint) as
            uint32_t as uint32_t;
    ::core::ptr::write_volatile(spixbase as *mut uint8_t, Data);
}
/* *
  * @brief  Transmits a Data through the SPIx/I2Sx peripheral.
  * @param  SPIx: To select the SPIx/I2Sx peripheral, where x can be: 1, 2 or 3 
  *         in SPI mode or 2 or 3 in I2S mode or I2Sxext for I2S full duplex mode.  
  * @param  Data: Data to be transmitted.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_SendData16(mut SPIx: *mut SPI_TypeDef,
                                            mut Data: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*SPIx).DR as *mut uint16_t, Data);
}
/* *
  * @brief  Returns the most recent received data by the SPIx peripheral. 
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @retval The value of the received data.
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_ReceiveData8(mut SPIx: *mut SPI_TypeDef)
 -> uint8_t {
    let mut spixbase: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    spixbase = SPIx as uint32_t;
    spixbase =
        (spixbase as
             libc::c_uint).wrapping_add(0xc as libc::c_int as libc::c_uint) as
            uint32_t as uint32_t;
    return *(spixbase as *mut uint8_t);
}
/* *
  * @brief  Returns the most recent received data by the SPIx peripheral. 
  * @param  SPIx: To select the SPIx/I2Sx peripheral, where x can be: 1, 2 or 3 
  *         in SPI mode or 2 or 3 in I2S mode or I2Sxext for I2S full duplex mode.
  * @retval The value of the received data.
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_ReceiveData16(mut SPIx: *mut SPI_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    return (*SPIx).DR;
}
/* *
  * @}
  */
/* * @defgroup SPI_Group3 Hardware CRC Calculation functions
 *  @brief   Hardware CRC Calculation functions
 *
@verbatim   
 ===============================================================================
                  ##### Hardware CRC Calculation functions #####
 ===============================================================================  
    [..] This section provides a set of functions allowing to manage the SPI CRC hardware 
         calculation.
    [..] SPI communication using CRC is possible through the following procedure:
         (#) Program the Data direction, Polarity, Phase, First Data, Baud Rate Prescaler, 
             Slave Management, Peripheral Mode and CRC Polynomial values using the SPI_Init()
             function.
         (#) Enable the CRC calculation using the SPI_CalculateCRC() function.
         (#) Enable the SPI using the SPI_Cmd() function
         (#) Before writing the last data to the TX buffer, set the CRCNext bit using the 
             SPI_TransmitCRC() function to indicate that after transmission of the last 
             data, the CRC should be transmitted.
         (#) After transmitting the last data, the SPI transmits the CRC. The SPI_CR1_CRCNEXT
             bit is reset. The CRC is also received and compared against the SPI_RXCRCR 
             value. 
             If the value does not match, the SPI_FLAG_CRCERR flag is set and an interrupt
             can be generated when the SPI_I2S_IT_ERR interrupt is enabled.
    [..]
    (@)
         (+@) It is advised to don't read the calculate CRC values during the communication.
         (+@) When the SPI is in slave mode, be careful to enable CRC calculation only 
              when the clock is stable, that is, when the clock is in the steady state. 
              If not, a wrong CRC calculation may be done. In fact, the CRC is sensitive 
              to the SCK slave input clock as soon as CRCEN is set, and this, whatever 
              the value of the SPE bit.
         (+@) With high bitrate frequencies, be careful when transmitting the CRC.
              As the number of used CPU cycles has to be as low as possible in the CRC 
              transfer phase, it is forbidden to call software functions in the CRC 
              transmission sequence to avoid errors in the last data and CRC reception. 
              In fact, CRCNEXT bit has to be written before the end of the transmission/reception 
              of the last data.
         (+@) For high bit rate frequencies, it is advised to use the DMA mode to avoid the
              degradation of the SPI speed performance due to CPU accesses impacting the 
              SPI bandwidth.
         (+@) When the STM32F30x are configured as slaves and the NSS hardware mode is 
              used, the NSS pin needs to be kept low between the data phase and the CRC 
              phase.
         (+@) When the SPI is configured in slave mode with the CRC feature enabled, CRC
              calculation takes place even if a high level is applied on the NSS pin. 
              This may happen for example in case of a multislave environment where the 
              communication master addresses slaves alternately.
         (+@) Between a slave deselection (high level on NSS) and a new slave selection 
              (low level on NSS), the CRC value should be cleared on both master and slave
              sides in order to resynchronize the master and slave for their respective 
              CRC calculation.
    [..]          
    (@) To clear the CRC, follow the procedure below:
         (#@) Disable SPI using the SPI_Cmd() function.
         (#@) Disable the CRC calculation using the SPI_CalculateCRC() function.
         (#@) Enable the CRC calculation using the SPI_CalculateCRC() function.
         (#@) Enable SPI using the SPI_Cmd() function.

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the CRC calculation length for the selected SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_CRCLength: specifies the SPI CRC calculation length.
  *   This parameter can be one of the following values:
  *     @arg SPI_CRCLength_8b: Set CRC Calculation to 8 bits
  *     @arg SPI_CRCLength_16b: Set CRC Calculation to 16 bits
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_CRCLengthConfig(mut SPIx: *mut SPI_TypeDef,
                                             mut SPI_CRCLength: uint16_t) {
    /* Check the parameters */
    /* Clear CRCL bit */
    ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(0x800 as libc::c_int as uint16_t as
                                           libc::c_int) as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /* Set new CRCL bit value */
    ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     SPI_CRCLength as libc::c_int) as uint16_t
                                    as uint16_t);
}
/* *
  * @brief  Enables or disables the CRC value calculation of the transferred bytes.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx CRC value calculation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_CalculateCRC(mut SPIx: *mut SPI_TypeDef,
                                          mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI CRC calculation */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x2000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected SPI CRC calculation */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x2000 as libc::c_int as uint16_t
                                               as libc::c_int) as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Transmits the SPIx CRC value.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_TransmitCRC(mut SPIx: *mut SPI_TypeDef) {
    /* Check the parameters */
    /* Enable the selected SPI CRC transmission */
    ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     0x1000 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
}
/* *
  * @brief  Returns the transmit or the receive CRC register value for the specified SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_CRC: specifies the CRC register to be read.
  *   This parameter can be one of the following values:
  *     @arg SPI_CRC_Tx: Selects Tx CRC register
  *     @arg SPI_CRC_Rx: Selects Rx CRC register
  * @retval The selected CRC register value..
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_GetCRC(mut SPIx: *mut SPI_TypeDef,
                                    mut SPI_CRC: uint8_t) -> uint16_t {
    let mut crcreg: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    if SPI_CRC as libc::c_int != 0x1 as libc::c_int as uint8_t as libc::c_int
       {
        /* Get the Tx CRC register */
        crcreg = (*SPIx).TXCRCR
    } else {
        /* Get the Rx CRC register */
        crcreg = (*SPIx).RXCRCR
    }
    /* Return the selected CRC register */
    return crcreg;
}
/* *
  * @brief  Returns the CRC Polynomial register value for the specified SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @retval The CRC Polynomial register value.
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_GetCRCPolynomial(mut SPIx: *mut SPI_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    /* Return the CRC polynomial register */
    return (*SPIx).CRCPR;
}
/* *
  * @}
  */
/* * @defgroup SPI_Group4 DMA transfers management functions
 *  @brief   DMA transfers management functions
  *
@verbatim   
 ===============================================================================
                  ##### DMA transfers management functions #####
 ===============================================================================

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the SPIx/I2Sx DMA interface.
  * @param  SPIx:To select the SPIx/I2Sx peripheral, where x can be: 1, 2 or 3 
  *         in SPI mode or 2 or 3 in I2S mode or I2Sxext for I2S full duplex mode. 
  * @param  SPI_I2S_DMAReq: specifies the SPI DMA transfer request to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg SPI_I2S_DMAReq_Tx: Tx buffer DMA transfer request
  *     @arg SPI_I2S_DMAReq_Rx: Rx buffer DMA transfer request
  * @param  NewState: new state of the selected SPI DMA transfer request.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_DMACmd(mut SPIx: *mut SPI_TypeDef,
                                        mut SPI_I2S_DMAReq: uint16_t,
                                        mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI DMA requests */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         SPI_I2S_DMAReq as libc::c_int) as
                                        uint16_t as uint16_t)
    } else {
        /* Disable the selected SPI DMA requests */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(SPI_I2S_DMAReq as libc::c_int) as
                                             uint16_t as libc::c_int) as
                                        uint16_t as uint16_t)
    };
}
/* *
  * @brief  Configures the number of data to transfer type(Even/Odd) for the DMA
  *         last transfers and for the selected SPI.
  * @note   This function have a meaning only if DMA mode is selected and if 
  *         the packing mode is used (data length <= 8 and DMA transfer size halfword)  
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_LastDMATransfer: specifies the SPI last DMA transfers state.
  *   This parameter can be one of the following values:
  *     @arg SPI_LastDMATransfer_TxEvenRxEven: Number of data for transmission Even
  *          and number of data for reception Even.
  *     @arg SPI_LastDMATransfer_TxOddRxEven: Number of data for transmission Odd
  *          and number of data for reception Even.
  *     @arg SPI_LastDMATransfer_TxEvenRxOdd: Number of data for transmission Even
  *          and number of data for reception Odd.
  *     @arg SPI_LastDMATransfer_TxOddRxOdd: RNumber of data for transmission Odd
  *          and number of data for reception Odd.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_LastDMATransferCmd(mut SPIx: *mut SPI_TypeDef,
                                                mut SPI_LastDMATransfer:
                                                    uint16_t) {
    /* Check the parameters */
    /* Clear LDMA_TX and LDMA_RX bits */
    ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0x9fff as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /* Set new LDMA_TX and LDMA_RX bits value */
    ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     SPI_LastDMATransfer as libc::c_int) as
                                    uint16_t as uint16_t);
}
/* *
  * @}
  */
/* * @defgroup SPI_Group5 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
  *
@verbatim   
 ===============================================================================
              ##### Interrupts and flags management functions #####
 ===============================================================================  
    [..] This section provides a set of functions allowing to configure the SPI/I2S 
         Interrupts sources and check or clear the flags or pending bits status.
         The user should identify which mode will be used in his application to manage 
         the communication: Polling mode, Interrupt mode or DMA mode. 
    
  *** Polling Mode ***
  ====================
    [..] In Polling Mode, the SPI/I2S communication can be managed by 9 flags:
         (#) SPI_I2S_FLAG_TXE : to indicate the status of the transmit buffer register.
         (#) SPI_I2S_FLAG_RXNE : to indicate the status of the receive buffer register.
         (#) SPI_I2S_FLAG_BSY : to indicate the state of the communication layer of the SPI.
         (#) SPI_FLAG_CRCERR : to indicate if a CRC Calculation error occur.              
         (#) SPI_FLAG_MODF : to indicate if a Mode Fault error occur.
         (#) SPI_I2S_FLAG_OVR : to indicate if an Overrun error occur.
         (#) SPI_I2S_FLAG_FRE: to indicate a Frame Format error occurs.
         (#) I2S_FLAG_UDR: to indicate an Underrun error occurs.
         (#) I2S_FLAG_CHSIDE: to indicate Channel Side.
    [..]
         (@) Do not use the BSY flag to handle each data transmission or reception.
             It is better to use the TXE and RXNE flags instead.
    [..] In this Mode it is advised to use the following functions:
         (+) FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
         (+) void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);

  *** Interrupt Mode ***
  ======================
    [..] In Interrupt Mode, the SPI/I2S communication can be managed by 3 interrupt sources
         and 5 pending bits: 
    [..] Pending Bits:
         (#) SPI_I2S_IT_TXE : to indicate the status of the transmit buffer register.
         (#) SPI_I2S_IT_RXNE : to indicate the status of the receive buffer register.
         (#) SPI_I2S_IT_OVR : to indicate if an Overrun error occur.
         (#) I2S_IT_UDR : to indicate an Underrun Error occurs.
         (#) SPI_I2S_FLAG_FRE : to indicate a Frame Format error occurs.
    [..] Interrupt Source:
         (#) SPI_I2S_IT_TXE: specifies the interrupt source for the Tx buffer empty 
             interrupt.  
         (#) SPI_I2S_IT_RXNE : specifies the interrupt source for the Rx buffer not 
             empty interrupt.
         (#) SPI_I2S_IT_ERR : specifies the interrupt source for the errors interrupt.
    [..] In this Mode it is advised to use the following functions:
         (+) void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
         (+) ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);

  *** FIFO Status ***
  ===================
    [..] It is possible to monitor the FIFO status when a transfer is ongoing using the
         following function:
         (+) uint32_t SPI_GetFIFOStatus(uint8_t SPI_FIFO_Direction); 

  *** DMA Mode ***
  ================
    [..] In DMA Mode, the SPI communication can be managed by 2 DMA Channel requests:
         (#) SPI_I2S_DMAReq_Tx: specifies the Tx buffer DMA transfer request.
         (#) SPI_I2S_DMAReq_Rx: specifies the Rx buffer DMA transfer request.
    [..] In this Mode it is advised to use the following function:
         (+) void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified SPI/I2S interrupts.
  * @param  SPIx: To select the SPIx/I2Sx peripheral, where x can be: 1, 2 or 3 
  *         in SPI mode or 2 or 3 in I2S mode or I2Sxext for I2S full duplex mode.  
  * @param  SPI_I2S_IT: specifies the SPI interrupt source to be enabled or disabled. 
  *   This parameter can be one of the following values:
  *     @arg SPI_I2S_IT_TXE: Tx buffer empty interrupt mask
  *     @arg SPI_I2S_IT_RXNE: Rx buffer not empty interrupt mask
  *     @arg SPI_I2S_IT_ERR: Error interrupt mask
  * @param  NewState: new state of the specified SPI interrupt.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_ITConfig(mut SPIx: *mut SPI_TypeDef,
                                          mut SPI_I2S_IT: uint8_t,
                                          mut NewState: FunctionalState) {
    let mut itpos: uint16_t = 0 as libc::c_int as uint16_t;
    let mut itmask: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    /* Get the SPI IT index */
    itpos = (SPI_I2S_IT as libc::c_int >> 4 as libc::c_int) as uint16_t;
    /* Set the IT mask */
    itmask =
        ((1 as libc::c_int as uint16_t as libc::c_int) <<
             itpos as libc::c_int) as uint16_t;
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI interrupt */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         itmask as libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected SPI interrupt */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(itmask as libc::c_int) as uint16_t
                                             as libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Returns the current SPIx Transmission FIFO filled level.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @retval The Transmission FIFO filling state.
  *   - SPI_TransmissionFIFOStatus_Empty: when FIFO is empty
  *   - SPI_TransmissionFIFOStatus_1QuarterFull: if more than 1 quarter-full.
  *   - SPI_TransmissionFIFOStatus_HalfFull: if more than 1 half-full.
  *   - SPI_TransmissionFIFOStatus_Full: when FIFO is full.
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_GetTransmissionFIFOStatus(mut SPIx:
                                                           *mut SPI_TypeDef)
 -> uint16_t {
    /* Get the SPIx Transmission FIFO level bits */
    return ((*SPIx).SR as libc::c_int &
                0x1800 as libc::c_int as uint16_t as libc::c_int) as uint16_t;
}
/* *
  * @brief  Returns the current SPIx Reception FIFO filled level.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @retval The Reception FIFO filling state.
  *   - SPI_ReceptionFIFOStatus_Empty: when FIFO is empty
  *   - SPI_ReceptionFIFOStatus_1QuarterFull: if more than 1 quarter-full.
  *   - SPI_ReceptionFIFOStatus_HalfFull: if more than 1 half-full.
  *   - SPI_ReceptionFIFOStatus_Full: when FIFO is full.
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_GetReceptionFIFOStatus(mut SPIx:
                                                        *mut SPI_TypeDef)
 -> uint16_t {
    /* Get the SPIx Reception FIFO level bits */
    return ((*SPIx).SR as libc::c_int &
                0x600 as libc::c_int as uint16_t as libc::c_int) as uint16_t;
}
/* *
  * @brief  Checks whether the specified SPI flag is set or not.
  * @param  SPIx: To select the SPIx/I2Sx peripheral, where x can be: 1, 2 or 3 
  *         in SPI mode or 2 or 3 in I2S mode or I2Sxext for I2S full duplex mode.  
  * @param  SPI_I2S_FLAG: specifies the SPI flag to check. 
  *   This parameter can be one of the following values:
  *     @arg SPI_I2S_FLAG_TXE: Transmit buffer empty flag.
  *     @arg SPI_I2S_FLAG_RXNE: Receive buffer not empty flag.
  *     @arg SPI_I2S_FLAG_BSY: Busy flag.
  *     @arg SPI_I2S_FLAG_OVR: Overrun flag.
  *     @arg SPI_I2S_FLAG_MODF: Mode Fault flag.
  *     @arg SPI_I2S_FLAG_CRCERR: CRC Error flag.
  *     @arg SPI_I2S_FLAG_FRE: TI frame format error flag.
  *     @arg I2S_FLAG_UDR: Underrun Error flag.
  *     @arg I2S_FLAG_CHSIDE: Channel Side flag.   
  * @retval The new state of SPI_I2S_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_GetFlagStatus(mut SPIx: *mut SPI_TypeDef,
                                               mut SPI_I2S_FLAG: uint16_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of the specified SPI flag */
    if (*SPIx).SR as libc::c_int & SPI_I2S_FLAG as libc::c_int !=
           RESET as libc::c_int as uint16_t as libc::c_int {
        /* SPI_I2S_FLAG is set */
        bitstatus = SET
    } else {
        /* SPI_I2S_FLAG is reset */
        bitstatus = RESET
    }
    /* Return the SPI_I2S_FLAG status */
    return bitstatus;
}
/* *
  * @brief  Clears the SPIx CRC Error (CRCERR) flag.
  * @param  SPIx: To select the SPIx/I2Sx peripheral, where x can be: 1, 2 or 3 
  *         in SPI mode or 2 or 3 in I2S mode or I2Sxext for I2S full duplex mode. 
  * @param  SPI_I2S_FLAG: specifies the SPI flag to clear. 
  *   This function clears only CRCERR flag.
  * @note OVR (OverRun error) flag is cleared by software sequence: a read 
  *       operation to SPI_DR register (SPI_I2S_ReceiveData()) followed by a read 
  *       operation to SPI_SR register (SPI_I2S_GetFlagStatus()).
  * @note MODF (Mode Fault) flag is cleared by software sequence: a read/write 
  *       operation to SPI_SR register (SPI_I2S_GetFlagStatus()) followed by a 
  *       write operation to SPI_CR1 register (SPI_Cmd() to enable the SPI).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_ClearFlag(mut SPIx: *mut SPI_TypeDef,
                                           mut SPI_I2S_FLAG: uint16_t) {
    /* Check the parameters */
    /* Clear the selected SPI CRC Error (CRCERR) flag */
    ::core::ptr::write_volatile(&mut (*SPIx).SR as *mut uint16_t,
                                !(SPI_I2S_FLAG as libc::c_int) as uint16_t);
}
/* *
  ******************************************************************************
  * @file    stm32f30x_spi.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the SPI 
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
/* * @addtogroup SPI
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  SPI Init structure definition  
  */
/* !< Specifies the SPI unidirectional or bidirectional data mode.
                                         This parameter can be a value of @ref SPI_data_direction */
/* !< Specifies the SPI mode (Master/Slave).
                                         This parameter can be a value of @ref SPI_mode */
/* !< Specifies the SPI data size.
                                         This parameter can be a value of @ref SPI_data_size */
/* !< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_Clock_Polarity */
/* !< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_Clock_Phase */
/* !< Specifies whether the NSS signal is managed by
                                         hardware (NSS pin) or by software using the SSI bit.
                                         This parameter can be a value of @ref SPI_Slave_Select_management */
/* !< Specifies the Baud Rate prescaler value which will be
                                         used to configure the transmit and receive SCK clock.
                                         This parameter can be a value of @ref SPI_BaudRate_Prescaler.
                                         @note The communication clock is derived from the master
                                               clock. The slave clock does not need to be set. */
/* !< Specifies whether data transfers start from MSB or LSB bit.
                                         This parameter can be a value of @ref SPI_MSB_LSB_transmission */
/* !< Specifies the polynomial used for the CRC calculation. */
/* * 
  * @brief  I2S Init structure definition  
  */
/* !< Specifies the I2S operating mode.
                                  This parameter can be a value of @ref I2S_Mode */
/* !< Specifies the standard used for the I2S communication.
                                  This parameter can be a value of @ref I2S_Standard */
/* !< Specifies the data format for the I2S communication.
                                  This parameter can be a value of @ref I2S_Data_Format */
/* !< Specifies whether the I2S MCLK output is enabled or not.
                                  This parameter can be a value of @ref I2S_MCLK_Output */
/* !< Specifies the frequency selected for the I2S communication.
                                  This parameter can be a value of @ref I2S_Audio_Frequency */
/* !< Specifies the idle state of the I2S clock.
                                  This parameter can be a value of @ref I2S_Clock_Polarity */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup SPI_Exported_Constants
  * @{
  */
/* * @defgroup SPI_data_direction 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_data_size
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_CRC_length
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Clock_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Clock_Phase 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Slave_Select_management 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_BaudRate_Prescaler 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_MSB_LSB_transmission 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_Standard 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_Data_Format 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_MCLK_Output 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_Audio_Frequency 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_Clock_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_FIFO_reception_threshold 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_I2S_DMA_transfer_requests 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_last_DMA_transfers
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_NSS_internal_software_management 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_CRC_Transmit_Receive 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_direction_transmit_receive 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_I2S_interrupts_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_transmission_fifo_status_level 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_reception_fifo_status_level 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_I2S_flags_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_CRC_polynomial 
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
/* Function used to set the SPI configuration to the default reset state*******/
/* Initialization and Configuration functions *********************************/
/* Data transfers functions ***************************************************/
/* Hardware CRC Calculation functions *****************************************/
/* DMA transfers management functions *****************************************/
/* Interrupts and flags management functions **********************************/
/* *
  * @brief  Checks whether the specified SPI/I2S interrupt has occurred or not.
  * @param  SPIx: To select the SPIx/I2Sx peripheral, where x can be: 1, 2 or 3 
  *         in SPI mode or 2 or 3 in I2S mode or I2Sxext for I2S full duplex mode.  
  * @param  SPI_I2S_IT: specifies the SPI interrupt source to check. 
  *   This parameter can be one of the following values:
  *     @arg SPI_I2S_IT_TXE: Transmit buffer empty interrupt.
  *     @arg SPI_I2S_IT_RXNE: Receive buffer not empty interrupt.
  *     @arg SPI_IT_MODF: Mode Fault interrupt.
  *     @arg SPI_I2S_IT_OVR: Overrun interrupt.
  *     @arg I2S_IT_UDR: Underrun interrupt.  
  *     @arg SPI_I2S_IT_FRE: Format Error interrupt.  
  * @retval The new state of SPI_I2S_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_GetITStatus(mut SPIx: *mut SPI_TypeDef,
                                             mut SPI_I2S_IT: uint8_t)
 -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut itpos: uint16_t = 0 as libc::c_int as uint16_t;
    let mut itmask: uint16_t = 0 as libc::c_int as uint16_t;
    let mut enablestatus: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    /* Get the SPI_I2S_IT index */
    itpos =
        ((0x1 as libc::c_int) <<
             (SPI_I2S_IT as libc::c_int & 0xf as libc::c_int)) as uint16_t;
    /* Get the SPI_I2S_IT IT mask */
    itmask = (SPI_I2S_IT as libc::c_int >> 4 as libc::c_int) as uint16_t;
    /* Set the IT mask */
    itmask = ((0x1 as libc::c_int) << itmask as libc::c_int) as uint16_t;
    /* Get the SPI_I2S_IT enable bit status */
    enablestatus =
        ((*SPIx).CR2 as libc::c_int & itmask as libc::c_int) as uint16_t;
    /* Check the status of the specified SPI interrupt */
    if (*SPIx).SR as libc::c_int & itpos as libc::c_int !=
           RESET as libc::c_int as uint16_t as libc::c_int &&
           enablestatus as libc::c_int != 0 {
        /* SPI_I2S_IT is set */
        bitstatus = SET
    } else {
        /* SPI_I2S_IT is reset */
        bitstatus = RESET
    }
    /* Return the SPI_I2S_IT status */
    return bitstatus;
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
