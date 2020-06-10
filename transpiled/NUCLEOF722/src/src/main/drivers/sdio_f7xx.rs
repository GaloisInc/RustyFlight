use ::libc;
extern "C" {
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn dmaGetIdentifier(stream: *const DMA_Stream_TypeDef) -> dmaIdentifier_e;
    #[no_mangle]
    fn dmaInit(identifier: dmaIdentifier_e, owner: resourceOwner_e,
               resourceIndex: uint8_t);
    #[no_mangle]
    fn dmaSetHandler(identifier: dmaIdentifier_e,
                     callback: dmaCallbackHandlerFuncPtr, priority: uint32_t,
                     userParam: uint32_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
pub type IRQn_Type = libc::c_int;
pub const SDMMC2_IRQn: IRQn_Type = 103;
pub const LPTIM1_IRQn: IRQn_Type = 93;
pub const QUADSPI_IRQn: IRQn_Type = 92;
pub const SAI2_IRQn: IRQn_Type = 91;
pub const SAI1_IRQn: IRQn_Type = 87;
pub const SPI5_IRQn: IRQn_Type = 85;
pub const SPI4_IRQn: IRQn_Type = 84;
pub const UART8_IRQn: IRQn_Type = 83;
pub const UART7_IRQn: IRQn_Type = 82;
pub const FPU_IRQn: IRQn_Type = 81;
pub const RNG_IRQn: IRQn_Type = 80;
pub const OTG_HS_IRQn: IRQn_Type = 77;
pub const OTG_HS_WKUP_IRQn: IRQn_Type = 76;
pub const OTG_HS_EP1_IN_IRQn: IRQn_Type = 75;
pub const OTG_HS_EP1_OUT_IRQn: IRQn_Type = 74;
pub const I2C3_ER_IRQn: IRQn_Type = 73;
pub const I2C3_EV_IRQn: IRQn_Type = 72;
pub const USART6_IRQn: IRQn_Type = 71;
pub const DMA2_Stream7_IRQn: IRQn_Type = 70;
pub const DMA2_Stream6_IRQn: IRQn_Type = 69;
pub const DMA2_Stream5_IRQn: IRQn_Type = 68;
pub const OTG_FS_IRQn: IRQn_Type = 67;
pub const ETH_WKUP_IRQn: IRQn_Type = 62;
pub const ETH_IRQn: IRQn_Type = 61;
pub const DMA2_Stream4_IRQn: IRQn_Type = 60;
pub const DMA2_Stream3_IRQn: IRQn_Type = 59;
pub const DMA2_Stream2_IRQn: IRQn_Type = 58;
pub const DMA2_Stream1_IRQn: IRQn_Type = 57;
pub const DMA2_Stream0_IRQn: IRQn_Type = 56;
pub const TIM7_IRQn: IRQn_Type = 55;
pub const TIM6_DAC_IRQn: IRQn_Type = 54;
pub const UART5_IRQn: IRQn_Type = 53;
pub const UART4_IRQn: IRQn_Type = 52;
pub const SPI3_IRQn: IRQn_Type = 51;
pub const TIM5_IRQn: IRQn_Type = 50;
pub const SDMMC1_IRQn: IRQn_Type = 49;
pub const FMC_IRQn: IRQn_Type = 48;
pub const DMA1_Stream7_IRQn: IRQn_Type = 47;
pub const TIM8_CC_IRQn: IRQn_Type = 46;
pub const TIM8_TRG_COM_TIM14_IRQn: IRQn_Type = 45;
pub const TIM8_UP_TIM13_IRQn: IRQn_Type = 44;
pub const TIM8_BRK_TIM12_IRQn: IRQn_Type = 43;
pub const OTG_FS_WKUP_IRQn: IRQn_Type = 42;
pub const RTC_Alarm_IRQn: IRQn_Type = 41;
pub const EXTI15_10_IRQn: IRQn_Type = 40;
pub const USART3_IRQn: IRQn_Type = 39;
pub const USART2_IRQn: IRQn_Type = 38;
pub const USART1_IRQn: IRQn_Type = 37;
pub const SPI2_IRQn: IRQn_Type = 36;
pub const SPI1_IRQn: IRQn_Type = 35;
pub const I2C2_ER_IRQn: IRQn_Type = 34;
pub const I2C2_EV_IRQn: IRQn_Type = 33;
pub const I2C1_ER_IRQn: IRQn_Type = 32;
pub const I2C1_EV_IRQn: IRQn_Type = 31;
pub const TIM4_IRQn: IRQn_Type = 30;
pub const TIM3_IRQn: IRQn_Type = 29;
pub const TIM2_IRQn: IRQn_Type = 28;
pub const TIM1_CC_IRQn: IRQn_Type = 27;
pub const TIM1_TRG_COM_TIM11_IRQn: IRQn_Type = 26;
pub const TIM1_UP_TIM10_IRQn: IRQn_Type = 25;
pub const TIM1_BRK_TIM9_IRQn: IRQn_Type = 24;
pub const EXTI9_5_IRQn: IRQn_Type = 23;
pub const CAN1_SCE_IRQn: IRQn_Type = 22;
pub const CAN1_RX1_IRQn: IRQn_Type = 21;
pub const CAN1_RX0_IRQn: IRQn_Type = 20;
pub const CAN1_TX_IRQn: IRQn_Type = 19;
pub const ADC_IRQn: IRQn_Type = 18;
pub const DMA1_Stream6_IRQn: IRQn_Type = 17;
pub const DMA1_Stream5_IRQn: IRQn_Type = 16;
pub const DMA1_Stream4_IRQn: IRQn_Type = 15;
pub const DMA1_Stream3_IRQn: IRQn_Type = 14;
pub const DMA1_Stream2_IRQn: IRQn_Type = 13;
pub const DMA1_Stream1_IRQn: IRQn_Type = 12;
pub const DMA1_Stream0_IRQn: IRQn_Type = 11;
pub const EXTI4_IRQn: IRQn_Type = 10;
pub const EXTI3_IRQn: IRQn_Type = 9;
pub const EXTI2_IRQn: IRQn_Type = 8;
pub const EXTI1_IRQn: IRQn_Type = 7;
pub const EXTI0_IRQn: IRQn_Type = 6;
pub const RCC_IRQn: IRQn_Type = 5;
pub const FLASH_IRQn: IRQn_Type = 4;
pub const RTC_WKUP_IRQn: IRQn_Type = 3;
pub const TAMP_STAMP_IRQn: IRQn_Type = 2;
pub const PVD_IRQn: IRQn_Type = 1;
pub const WWDG_IRQn: IRQn_Type = 0;
pub const SysTick_IRQn: IRQn_Type = -1;
pub const PendSV_IRQn: IRQn_Type = -2;
pub const DebugMonitor_IRQn: IRQn_Type = -4;
pub const SVCall_IRQn: IRQn_Type = -5;
pub const UsageFault_IRQn: IRQn_Type = -10;
pub const BusFault_IRQn: IRQn_Type = -11;
pub const MemoryManagement_IRQn: IRQn_Type = -12;
pub const NonMaskableInt_IRQn: IRQn_Type = -14;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct NVIC_Type {
    pub ISER: [uint32_t; 8],
    pub RESERVED0: [uint32_t; 24],
    pub ICER: [uint32_t; 8],
    pub RSERVED1: [uint32_t; 24],
    pub ISPR: [uint32_t; 8],
    pub RESERVED2: [uint32_t; 24],
    pub ICPR: [uint32_t; 8],
    pub RESERVED3: [uint32_t; 24],
    pub IABR: [uint32_t; 8],
    pub RESERVED4: [uint32_t; 56],
    pub IP: [uint8_t; 240],
    pub RESERVED5: [uint32_t; 644],
    pub STIR: uint32_t,
}
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SDMMC_TypeDef {
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
/*
 * Original author: Alain (https://github.com/aroyer-qc)
 * Modified for F4 and BF source: Chris Hockuba (https://github.com/conkerkh)
 *
 */
/* SDCARD pinouts
 *
 * SD CARD PINS
   _________________
  / 1 2 3 4 5 6 7 8 |  NR   |SDIO INTERFACE
 /                  |       |NAME     STM32F746     DESCRIPTION
/ 9                 |       |         4-BIT  1-BIT
|                   |       |
|                   |   1   |CD/DAT3  PC11   -      Connector data line 3
|                   |   2   |CMD      PD2    PD2    Command/Response line
|                   |   3   |VSS1     GND    GND    GND
|   SD CARD Pinout  |   4   |VDD      3.3V   3.3V   3.3V Power supply
|                   |   5   |CLK      PC12   PC12   Clock
|                   |   6   |VSS2     GND    GND    GND
|                   |   7   |DAT0     PC8    PC8    Connector data line 0
|                   |   8   |DAT1     PC9    -      Connector data line 1
|___________________|   9   |DAT2     PC10   -      Connector data line 2

 */
/* Define(s) --------------------------------------------------------------------------------------------------------*/
//#define MSD_OK                        		    ((uint8_t)0x00)
/* Structure(s) -----------------------------------------------------------------------------------------------------*/
pub type SD_Error_t = libc::c_uint;
pub const SD_OK: SD_Error_t = 0;
pub const SD_BUSY: SD_Error_t = 43;
pub const SD_ERROR: SD_Error_t = 42;
pub const SD_UNSUPPORTED_HW: SD_Error_t = 41;
pub const SD_UNSUPPORTED_FEATURE: SD_Error_t = 40;
pub const SD_INVALID_PARAMETER: SD_Error_t = 39;
pub const SD_REQUEST_NOT_APPLICABLE: SD_Error_t = 38;
pub const SD_REQUEST_PENDING: SD_Error_t = 37;
pub const SD_NOT_CONFIGURED: SD_Error_t = 36;
// Standard error defines
pub const SD_INTERNAL_ERROR: SD_Error_t = 35;
pub const SD_OUT_OF_BOUND: SD_Error_t = 34;
pub const SD_SDMMC_UNKNOWN_FUNCTION: SD_Error_t = 33;
pub const SD_SDMMC_FUNCTION_FAILED: SD_Error_t = 32;
pub const SD_SDMMC_FUNCTION_BUSY: SD_Error_t = 31;
pub const SD_SDMMC_DISABLED: SD_Error_t = 30;
pub const SD_SWITCH_ERROR: SD_Error_t = 29;
pub const SD_ADDR_OUT_OF_RANGE: SD_Error_t = 28;
// Error in sequence of authentication.
pub const SD_INVALID_VOLTRANGE: SD_Error_t = 27;
// Erase sequence was cleared before executing because an out of erase sequence command was received
pub const SD_AKE_SEQ_ERROR: SD_Error_t = 26;
// Command has been executed without using internal ECC
pub const SD_ERASE_RESET: SD_Error_t = 25;
// Only partial address space was erased
pub const SD_CARD_ECC_DISABLED: SD_Error_t = 24;
// CID/CSD overwrite error
pub const SD_WP_ERASE_SKIP: SD_Error_t = 23;
// The card could not sustain data programming in stream mode
pub const SD_CID_CSD_OVERWRITE: SD_Error_t = 22;
// The card could not sustain data transfer in stream read operation.
pub const SD_STREAM_WRITE_OVERRUN: SD_Error_t = 21;
// General or unknown error
pub const SD_STREAM_READ_UNDERRUN: SD_Error_t = 20;
// Internal card controller error
pub const SD_GENERAL_UNKNOWN_ERROR: SD_Error_t = 19;
// Card internal ECC was applied but failed to correct the data
pub const SD_CC_ERROR: SD_Error_t = 18;
// Command is not legal for the card state
pub const SD_CARD_ECC_FAILED: SD_Error_t = 17;
// CRC check of the previous command failed
pub const SD_ILLEGAL_CMD: SD_Error_t = 16;
// Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card
pub const SD_COM_CRC_FAILED: SD_Error_t = 15;
// Attempt to program a write protect block
pub const SD_LOCK_UNLOCK_FAILED: SD_Error_t = 14;
// An invalid selection for erase groups
pub const SD_WRITE_PROT_VIOLATION: SD_Error_t = 13;
// An error in the sequence of erase command occurs.
pub const SD_BAD_ERASE_PARAM: SD_Error_t = 12;
// Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length
pub const SD_ERASE_SEQ_ERR: SD_Error_t = 11;
// Misaligned address
pub const SD_BLOCK_LEN_ERR: SD_Error_t = 10;
// Command's argument was out of range.
pub const SD_ADDR_MISALIGNED: SD_Error_t = 9;
// Start bit not detected on all data signals in wide bus mode
pub const SD_CMD_OUT_OF_RANGE: SD_Error_t = 8;
// Receive FIFO overrun
pub const SD_START_BIT_ERR: SD_Error_t = 7;
// Transmit FIFO underrun
pub const SD_RX_OVERRUN: SD_Error_t = 6;
// Data TimeOut
pub const SD_TX_UNDERRUN: SD_Error_t = 5;
// Command response TimeOut
pub const SD_DATA_TIMEOUT: SD_Error_t = 4;
// Data block sent/received (CRC check failed)
pub const SD_CMD_RSP_TIMEOUT: SD_Error_t = 3;
// Command response received (but CRC check failed)
pub const SD_DATA_CRC_FAIL: SD_Error_t = 2;
// SD specific error defines
pub const SD_CMD_CRC_FAIL: SD_Error_t = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SD_CardStatus_t {
    pub DAT_BUS_WIDTH: uint8_t,
    pub SECURED_MODE: uint8_t,
    pub SD_CARD_TYPE: uint16_t,
    pub SIZE_OF_PROTECTED_AREA: uint32_t,
    pub SPEED_CLASS: uint8_t,
    pub PERFORMANCE_MOVE: uint8_t,
    pub AU_SIZE: uint8_t,
    pub ERASE_SIZE: uint16_t,
    pub ERASE_TIMEOUT: uint8_t,
    pub ERASE_OFFSET: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SD_CSD_t {
    pub CSDStruct: uint8_t,
    pub SysSpecVersion: uint8_t,
    pub Reserved1: uint8_t,
    pub TAAC: uint8_t,
    pub NSAC: uint8_t,
    pub MaxBusClkFrec: uint8_t,
    pub CardComdClasses: uint16_t,
    pub RdBlockLen: uint8_t,
    pub PartBlockRead: uint8_t,
    pub WrBlockMisalign: uint8_t,
    pub RdBlockMisalign: uint8_t,
    pub DSRImpl: uint8_t,
    pub Reserved2: uint8_t,
    pub DeviceSize: uint32_t,
    pub MaxRdCurrentVDDMin: uint8_t,
    pub MaxRdCurrentVDDMax: uint8_t,
    pub MaxWrCurrentVDDMin: uint8_t,
    pub MaxWrCurrentVDDMax: uint8_t,
    pub DeviceSizeMul: uint8_t,
    pub EraseGrSize: uint8_t,
    pub EraseGrMul: uint8_t,
    pub WrProtectGrSize: uint8_t,
    pub WrProtectGrEnable: uint8_t,
    pub ManDeflECC: uint8_t,
    pub WrSpeedFact: uint8_t,
    pub MaxWrBlockLen: uint8_t,
    pub WriteBlockPaPartial: uint8_t,
    pub Reserved3: uint8_t,
    pub ContentProtectAppli: uint8_t,
    pub FileFormatGrouop: uint8_t,
    pub CopyFlag: uint8_t,
    pub PermWrProtect: uint8_t,
    pub TempWrProtect: uint8_t,
    pub FileFormat: uint8_t,
    pub ECC: uint8_t,
    pub CSD_CRC: uint8_t,
    pub Reserved4: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SD_CID_t {
    pub ManufacturerID: uint8_t,
    pub OEM_AppliID: uint16_t,
    pub ProdName1: uint32_t,
    pub ProdName2: uint8_t,
    pub ProdRev: uint8_t,
    pub ProdSN: uint32_t,
    pub Reserved1: uint8_t,
    pub ManufactDate: uint16_t,
    pub CID_CRC: uint8_t,
    pub Reserved2: uint8_t,
}
pub type SD_CardType_t = libc::c_uint;
pub const SD_HIGH_CAPACITY_MMC: SD_CardType_t = 7;
pub const SD_SECURE_DIGITAL_IO_COMBO: SD_CardType_t = 6;
pub const SD_HIGH_SPEED_MULTIMEDIA: SD_CardType_t = 5;
pub const SD_SECURE_DIGITAL_IO: SD_CardType_t = 4;
pub const SD_MULTIMEDIA: SD_CardType_t = 3;
pub const SD_HIGH_CAPACITY: SD_CardType_t = 2;
pub const SD_STD_CAPACITY_V2_0: SD_CardType_t = 1;
pub const SD_STD_CAPACITY_V1_1: SD_CardType_t = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SD_CardInfo_t {
    pub SD_csd: SD_CSD_t,
    pub SD_cid: SD_CID_t,
    pub CardCapacity: uint64_t,
    pub CardBlockSize: uint32_t,
}
pub type dmaChannelDescriptor_t = dmaChannelDescriptor_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct dmaChannelDescriptor_s {
    pub dma: *mut DMA_TypeDef,
    pub ref_0: *mut DMA_Stream_TypeDef,
    pub stream: uint8_t,
    pub irqHandlerCallback: dmaCallbackHandlerFuncPtr,
    pub flagsShift: uint8_t,
    pub irqN: IRQn_Type,
    pub userParam: uint32_t,
    pub owner: resourceOwner_e,
    pub resourceIndex: uint8_t,
    pub completeFlag: uint32_t,
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
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
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
pub type dmaCallbackHandlerFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut dmaChannelDescriptor_s) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SD_Handle_t {
    pub CSD: [uint32_t; 4],
    pub CID: [uint32_t; 4],
    pub TransferComplete: uint32_t,
    pub TransferError: uint32_t,
    pub RXCplt: uint32_t,
    pub TXCplt: uint32_t,
    pub Operation: uint32_t,
}
pub const SD_MULTIPLE_BLOCK: C2RustUnnamed = 1;
pub type dmaIdentifier_e = libc::c_uint;
pub const DMA_LAST_HANDLER: dmaIdentifier_e = 16;
pub const DMA2_ST7_HANDLER: dmaIdentifier_e = 16;
pub const DMA2_ST6_HANDLER: dmaIdentifier_e = 15;
pub const DMA2_ST5_HANDLER: dmaIdentifier_e = 14;
pub const DMA2_ST4_HANDLER: dmaIdentifier_e = 13;
pub const DMA2_ST3_HANDLER: dmaIdentifier_e = 12;
pub const DMA2_ST2_HANDLER: dmaIdentifier_e = 11;
pub const DMA2_ST1_HANDLER: dmaIdentifier_e = 10;
pub const DMA2_ST0_HANDLER: dmaIdentifier_e = 9;
pub const DMA1_ST7_HANDLER: dmaIdentifier_e = 8;
pub const DMA1_ST6_HANDLER: dmaIdentifier_e = 7;
pub const DMA1_ST5_HANDLER: dmaIdentifier_e = 6;
pub const DMA1_ST4_HANDLER: dmaIdentifier_e = 5;
pub const DMA1_ST3_HANDLER: dmaIdentifier_e = 4;
pub const DMA1_ST2_HANDLER: dmaIdentifier_e = 3;
pub const DMA1_ST1_HANDLER: dmaIdentifier_e = 2;
pub const DMA1_ST0_HANDLER: dmaIdentifier_e = 1;
pub const DMA_NONE: dmaIdentifier_e = 0;
// type specifying IO pin. Currently ioRec_t pointer, but this may change
// NONE initializer for ioTag_t variables
// NONE initializer for IO_t variable
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
pub type timeMs_t = uint32_t;
pub const SD_CARD_ERROR: SD_CardState_t = 255;
pub type SD_CardState_t = libc::c_uint;
pub const SD_CARD_DISCONNECTED: SD_CardState_t = 8;
pub const SD_CARD_PROGRAMMING: SD_CardState_t = 7;
pub const SD_CARD_RECEIVING: SD_CardState_t = 6;
pub const SD_CARD_SENDING: SD_CardState_t = 5;
pub const SD_CARD_TRANSFER: SD_CardState_t = 4;
pub const SD_CARD_STANDBY: SD_CardState_t = 3;
pub const SD_CARD_IDENTIFICATION: SD_CardState_t = 2;
pub const SD_CARD_READY: SD_CardState_t = 1;
pub const SD_SINGLE_BLOCK: C2RustUnnamed = 0;
pub type C2RustUnnamed = libc::c_uint;
#[inline]
unsafe extern "C" fn __NVIC_GetPriorityGrouping() -> uint32_t {
    return (((*((0xe000e000 as
                     libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong) as
                    *mut SCB_Type)).AIRCR as libc::c_ulong &
                 (7 as libc::c_ulong) << 8 as libc::c_uint) >>
                8 as libc::c_uint) as uint32_t;
}
#[inline]
unsafe extern "C" fn __NVIC_EnableIRQ(mut IRQn: IRQn_Type) {
    if IRQn as int32_t >= 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0x100
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut NVIC_Type)).ISER[(IRQn as
                                                                           uint32_t
                                                                           >>
                                                                           5
                                                                               as
                                                                               libc::c_ulong)
                                                                          as
                                                                          usize]
                                        as *mut uint32_t,
                                    ((1 as libc::c_ulong) <<
                                         (IRQn as uint32_t as libc::c_ulong &
                                              0x1f as libc::c_ulong)) as
                                        uint32_t)
    };
}
#[inline]
unsafe extern "C" fn __NVIC_SetPriority(mut IRQn: IRQn_Type,
                                        mut priority: uint32_t) {
    if IRQn as int32_t >= 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0x100
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut NVIC_Type)).IP[IRQn as
                                                                        uint32_t
                                                                        as
                                                                        usize]
                                        as *mut uint8_t,
                                    (priority <<
                                         (8 as
                                              libc::c_uint).wrapping_sub(4 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                                         & 0xff as libc::c_ulong as uint32_t)
                                        as uint8_t)
    } else {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0xd00
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut SCB_Type)).SHPR[(IRQn as
                                                                          uint32_t
                                                                          as
                                                                          libc::c_ulong
                                                                          &
                                                                          0xf
                                                                              as
                                                                              libc::c_ulong).wrapping_sub(4
                                                                                                              as
                                                                                                              libc::c_ulong)
                                                                         as
                                                                         usize]
                                        as *mut uint8_t,
                                    (priority <<
                                         (8 as
                                              libc::c_uint).wrapping_sub(4 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                                         & 0xff as libc::c_ulong as uint32_t)
                                        as uint8_t)
    };
}
#[inline]
unsafe extern "C" fn NVIC_EncodePriority(mut PriorityGroup: uint32_t,
                                         mut PreemptPriority: uint32_t,
                                         mut SubPriority: uint32_t)
 -> uint32_t {
    let mut PriorityGroupTmp: uint32_t =
        PriorityGroup & 0x7 as libc::c_ulong as uint32_t;
    let mut PreemptPriorityBits: uint32_t = 0;
    let mut SubPriorityBits: uint32_t = 0;
    PreemptPriorityBits =
        if (7 as
                libc::c_ulong).wrapping_sub(PriorityGroupTmp as libc::c_ulong)
               > 4 as libc::c_int as uint32_t as libc::c_ulong {
            4 as libc::c_int as uint32_t
        } else {
            (7 as
                 libc::c_ulong).wrapping_sub(PriorityGroupTmp as
                                                 libc::c_ulong) as uint32_t
        };
    SubPriorityBits =
        if PriorityGroupTmp.wrapping_add(4 as libc::c_int as uint32_t) <
               7 as libc::c_ulong as uint32_t {
            0 as libc::c_ulong as uint32_t
        } else {
            (PriorityGroupTmp as
                 libc::c_ulong).wrapping_sub(7 as
                                                 libc::c_ulong).wrapping_add(4
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 uint32_t
                                                                                 as
                                                                                 libc::c_ulong)
                as uint32_t
        };
    return (PreemptPriority &
                ((1 as libc::c_ulong) <<
                     PreemptPriorityBits).wrapping_sub(1 as libc::c_ulong) as
                    uint32_t) << SubPriorityBits |
               SubPriority &
                   ((1 as libc::c_ulong) <<
                        SubPriorityBits).wrapping_sub(1 as libc::c_ulong) as
                       uint32_t;
}
/* Variable(s) ------------------------------------------------------------------------------------------------------*/
static mut SD_Handle: SD_Handle_t =
    SD_Handle_t{CSD: [0; 4],
                CID: [0; 4],
                TransferComplete: 0,
                TransferError: 0,
                RXCplt: 0,
                TXCplt: 0,
                Operation: 0,};
#[no_mangle]
pub static mut SD_CardInfo: SD_CardInfo_t =
    SD_CardInfo_t{SD_csd:
                      SD_CSD_t{CSDStruct: 0,
                               SysSpecVersion: 0,
                               Reserved1: 0,
                               TAAC: 0,
                               NSAC: 0,
                               MaxBusClkFrec: 0,
                               CardComdClasses: 0,
                               RdBlockLen: 0,
                               PartBlockRead: 0,
                               WrBlockMisalign: 0,
                               RdBlockMisalign: 0,
                               DSRImpl: 0,
                               Reserved2: 0,
                               DeviceSize: 0,
                               MaxRdCurrentVDDMin: 0,
                               MaxRdCurrentVDDMax: 0,
                               MaxWrCurrentVDDMin: 0,
                               MaxWrCurrentVDDMax: 0,
                               DeviceSizeMul: 0,
                               EraseGrSize: 0,
                               EraseGrMul: 0,
                               WrProtectGrSize: 0,
                               WrProtectGrEnable: 0,
                               ManDeflECC: 0,
                               WrSpeedFact: 0,
                               MaxWrBlockLen: 0,
                               WriteBlockPaPartial: 0,
                               Reserved3: 0,
                               ContentProtectAppli: 0,
                               FileFormatGrouop: 0,
                               CopyFlag: 0,
                               PermWrProtect: 0,
                               TempWrProtect: 0,
                               FileFormat: 0,
                               ECC: 0,
                               CSD_CRC: 0,
                               Reserved4: 0,},
                  SD_cid:
                      SD_CID_t{ManufacturerID: 0,
                               OEM_AppliID: 0,
                               ProdName1: 0,
                               ProdName2: 0,
                               ProdRev: 0,
                               ProdSN: 0,
                               Reserved1: 0,
                               ManufactDate: 0,
                               CID_CRC: 0,
                               Reserved2: 0,},
                  CardCapacity: 0,
                  CardBlockSize: 0,};
static mut SD_Status: uint32_t = 0;
static mut SD_CardRCA: uint32_t = 0;
#[no_mangle]
pub static mut SD_CardType: SD_CardType_t = SD_STD_CAPACITY_V1_1;
#[no_mangle]
pub static mut dma_stream: *mut DMA_Stream_TypeDef =
    0 as *const DMA_Stream_TypeDef as *mut DMA_Stream_TypeDef;
//static void             SD_PowerOFF                 (void);
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *		SD_IsDetected
  *
  * @brief  Test if card is present
  * @param  bool   true or false
  */
#[no_mangle]
pub unsafe extern "C" fn SD_IsDetected() -> bool {
    let mut status: uint8_t = 0x1 as libc::c_int as uint8_t;
    /* !< Check GPIO to detect SD */
    return status != 0;
}
/* Private function(s) ----------------------------------------------------------------------------------------------*/
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *		DataTransferInit
  *
  * @brief  Prepare the state machine for transfer
  * @param  SD_TransferType_e   TransfertDir
  * @param  SD_CARD_BlockSize_e Size
  */
unsafe extern "C" fn SD_DataTransferInit(mut Size: uint32_t,
                                         mut DataBlockSize: uint32_t,
                                         mut IsItReadFromCard: bool) {
    let mut Direction: uint32_t = 0; // Set the SDMMC1 Data TimeOut value
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).DTIMER as
                                    *mut uint32_t,
                                100000000 as libc::c_int as
                                    uint32_t); // Set the SDMMC1 DataLength value
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).DLEN as
                                    *mut uint32_t, Size);
    Direction =
        if IsItReadFromCard as libc::c_int == 1 as libc::c_int {
            ((0x1 as libc::c_uint)) << 1 as libc::c_uint
        } else { 0 as libc::c_int as libc::c_uint };
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2c00
                                                                              as
                                                                              libc::c_uint)
               as *mut SDMMC_TypeDef)).DCTRL;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (DataBlockSize | Direction |
                                          (0x1 as libc::c_uint) <<
                                              0 as libc::c_uint |
                                          0x1 as libc::c_int as libc::c_uint))
                                    as uint32_t as uint32_t);
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *		SD_TransmitCommand
  *
  * @brief  Send the commande to SDMMC1
  * @param  uint32_t Command
  * @param  uint32_t Argument              Must provide the response size
  * @param  uint8_t ResponseType
  * @retval SD Card error state
  */
unsafe extern "C" fn SD_TransmitCommand(mut Command: uint32_t,
                                        mut Argument: uint32_t,
                                        mut ResponseType: int8_t)
 -> SD_Error_t {
    let mut ErrorState: SD_Error_t = SD_OK; // Clear the Command Flags
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).ICR as
                                    *mut uint32_t,
                                (0x1 as libc::c_uint) << 0 as libc::c_uint |
                                    (0x1 as libc::c_uint) << 1 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 2 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 3 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 4 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 5 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 6 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 7 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 8 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) <<
                                        10 as
                                            libc::c_uint); // Set the SDMMC1 Argument value
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).ARG as
                                    *mut uint32_t,
                                Argument); // Set SDMMC1 command parameters
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).CMD as
                                    *mut uint32_t,
                                Command |
                                    (0x1 as libc::c_uint) <<
                                        10 as
                                            libc::c_uint); // Go idle command
    if Argument == 0 as libc::c_int as libc::c_uint &&
           ResponseType as libc::c_int == 0 as libc::c_int {
        ResponseType = -(1 as libc::c_int) as int8_t
    } // Clear the Command Flags
    ErrorState =
        SD_CmdResponse((Command & (0x3f as libc::c_uint) << 0 as libc::c_uint)
                           as uint8_t, ResponseType);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).ICR as
                                    *mut uint32_t,
                                (0x1 as libc::c_uint) << 0 as libc::c_uint |
                                    (0x1 as libc::c_uint) << 1 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 2 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 3 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 4 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 5 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 6 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 7 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 8 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) <<
                                        10 as libc::c_uint);
    return ErrorState;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Checks for error conditions for any response.
  *                                     - R2 (CID or CSD) response.
  *                                     - R3 (OCR) response.
  *
  * @param  SD_CMD: The sent command Index
  * @retval SD Card error state
  */
unsafe extern "C" fn SD_CmdResponse(mut SD_CMD: uint8_t,
                                    mut ResponseType: int8_t) -> SD_Error_t {
    let mut Response_R1: uint32_t =
        0; // Card is not V2.0 compliant or card does not support the set voltage range
    let mut TimeOut: uint32_t = 0;
    let mut Flag: uint32_t = 0;
    if ResponseType as libc::c_int == -(1 as libc::c_int) {
        Flag = (0x1 as libc::c_uint) << 7 as libc::c_uint
    } else {
        Flag =
            (0x1 as libc::c_uint) << 0 as libc::c_uint |
                (0x1 as libc::c_uint) << 6 as libc::c_uint |
                (0x1 as libc::c_uint) << 2 as libc::c_uint
    }
    TimeOut = 0x20000 as libc::c_int as uint32_t;
    loop  {
        SD_Status =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut SDMMC_TypeDef)).STA;
        TimeOut = TimeOut.wrapping_sub(1);
        if !(SD_Status & Flag == 0 as libc::c_int as libc::c_uint &&
                 TimeOut > 0 as libc::c_int as libc::c_uint) {
            break ;
        }
    }
    if ResponseType as libc::c_int <= 0 as libc::c_int {
        if TimeOut == 0 as libc::c_int as libc::c_uint {
            return SD_CMD_RSP_TIMEOUT
        } else { return SD_OK }
    }
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x10000 as
                                              libc::c_uint).wrapping_add(0x2c00
                                                                             as
                                                                             libc::c_uint)
              as *mut SDMMC_TypeDef)).STA &
           (0x1 as libc::c_uint) << 2 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        return SD_CMD_RSP_TIMEOUT
    }
    if ResponseType as libc::c_int == 3 as libc::c_int {
        if TimeOut == 0 as libc::c_int as libc::c_uint {
            return SD_CMD_RSP_TIMEOUT
        } else { return SD_OK }
        // Card is SD V2.0 compliant
    } // Check if response is of desired command
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x10000 as
                                              libc::c_uint).wrapping_add(0x2c00
                                                                             as
                                                                             libc::c_uint)
              as *mut SDMMC_TypeDef)).STA &
           (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        return SD_CMD_CRC_FAIL
    } // We have received response, retrieve it for analysis
    if ResponseType as libc::c_int == 2 as libc::c_int { return SD_OK }
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x10000 as
                                              libc::c_uint).wrapping_add(0x2c00
                                                                             as
                                                                             libc::c_uint)
              as *mut SDMMC_TypeDef)).RESPCMD as uint8_t as libc::c_int !=
           SD_CMD as libc::c_int {
        return SD_ILLEGAL_CMD
    }
    Response_R1 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2c00
                                                                              as
                                                                              libc::c_uint)
               as *mut SDMMC_TypeDef)).RESP1;
    if ResponseType as libc::c_int == 1 as libc::c_int {
        return CheckOCR_Response(Response_R1)
    } else {
        if ResponseType as libc::c_int == 6 as libc::c_int {
            if Response_R1 &
                   (0x2000 as libc::c_int as uint32_t |
                        0x4000 as libc::c_int as uint32_t |
                        0x8000 as libc::c_int as uint32_t) ==
                   0 as libc::c_int as uint32_t {
                SD_CardRCA = Response_R1
            }
            if Response_R1 & 0x2000 as libc::c_int as uint32_t ==
                   0x2000 as libc::c_int as uint32_t {
                return SD_GENERAL_UNKNOWN_ERROR
            }
            if Response_R1 & 0x4000 as libc::c_int as uint32_t ==
                   0x4000 as libc::c_int as uint32_t {
                return SD_ILLEGAL_CMD
            }
            if Response_R1 & 0x8000 as libc::c_int as uint32_t ==
                   0x8000 as libc::c_int as uint32_t {
                return SD_COM_CRC_FAILED
            }
        }
    }
    return SD_OK;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Analyze the OCR response and return the appropriate error code
  * @param  Response_R1: OCR Response code
  * @retval SD Card error state
  */
unsafe extern "C" fn CheckOCR_Response(mut Response_R1: uint32_t)
 -> SD_Error_t {
    if Response_R1 & 0xfdffe008 as libc::c_uint ==
           0 as libc::c_int as uint32_t {
        return SD_OK
    }
    if Response_R1 & 0x80000000 as libc::c_uint == 0x80000000 as libc::c_uint
       {
        return SD_ADDR_OUT_OF_RANGE
    }
    if Response_R1 & 0x40000000 as libc::c_int as uint32_t ==
           0x40000000 as libc::c_int as uint32_t {
        return SD_ADDR_MISALIGNED
    }
    if Response_R1 & 0x20000000 as libc::c_int as uint32_t ==
           0x20000000 as libc::c_int as uint32_t {
        return SD_BLOCK_LEN_ERR
    }
    if Response_R1 & 0x10000000 as libc::c_int as uint32_t ==
           0x10000000 as libc::c_int as uint32_t {
        return SD_ERASE_SEQ_ERR
    }
    if Response_R1 & 0x8000000 as libc::c_int as uint32_t ==
           0x8000000 as libc::c_int as uint32_t {
        return SD_BAD_ERASE_PARAM
    }
    if Response_R1 & 0x4000000 as libc::c_int as uint32_t ==
           0x4000000 as libc::c_int as uint32_t {
        return SD_WRITE_PROT_VIOLATION
    }
    if Response_R1 & 0x1000000 as libc::c_int as uint32_t ==
           0x1000000 as libc::c_int as uint32_t {
        return SD_LOCK_UNLOCK_FAILED
    }
    if Response_R1 & 0x800000 as libc::c_int as uint32_t ==
           0x800000 as libc::c_int as uint32_t {
        return SD_COM_CRC_FAILED
    }
    if Response_R1 & 0x400000 as libc::c_int as uint32_t ==
           0x400000 as libc::c_int as uint32_t {
        return SD_ILLEGAL_CMD
    }
    if Response_R1 & 0x200000 as libc::c_int as uint32_t ==
           0x200000 as libc::c_int as uint32_t {
        return SD_CARD_ECC_FAILED
    }
    if Response_R1 & 0x100000 as libc::c_int as uint32_t ==
           0x100000 as libc::c_int as uint32_t {
        return SD_CC_ERROR
    }
    if Response_R1 & 0x80000 as libc::c_int as uint32_t ==
           0x80000 as libc::c_int as uint32_t {
        return SD_GENERAL_UNKNOWN_ERROR
    }
    if Response_R1 & 0x40000 as libc::c_int as uint32_t ==
           0x40000 as libc::c_int as uint32_t {
        return SD_STREAM_READ_UNDERRUN
    }
    if Response_R1 & 0x20000 as libc::c_int as uint32_t ==
           0x20000 as libc::c_int as uint32_t {
        return SD_STREAM_WRITE_OVERRUN
    }
    if Response_R1 & 0x10000 as libc::c_int as uint32_t ==
           0x10000 as libc::c_int as uint32_t {
        return SD_CID_CSD_OVERWRITE
    }
    if Response_R1 & 0x8000 as libc::c_int as uint32_t ==
           0x8000 as libc::c_int as uint32_t {
        return SD_WP_ERASE_SKIP
    }
    if Response_R1 & 0x4000 as libc::c_int as uint32_t ==
           0x4000 as libc::c_int as uint32_t {
        return SD_CARD_ECC_DISABLED
    }
    if Response_R1 & 0x2000 as libc::c_int as uint32_t ==
           0x2000 as libc::c_int as uint32_t {
        return SD_ERASE_RESET
    }
    if Response_R1 & 0x8 as libc::c_int as uint32_t ==
           0x8 as libc::c_int as uint32_t {
        return SD_AKE_SEQ_ERROR
    }
    return SD_OK;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *		GetResponse
  *
  * @brief  Get response from SD device
  * @param  uint32_t*       pResponse
  */
unsafe extern "C" fn SD_GetResponse(mut pResponse: *mut uint32_t) {
    *pResponse.offset(0 as libc::c_int as isize) =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2c00
                                                                              as
                                                                              libc::c_uint)
               as *mut SDMMC_TypeDef)).RESP1;
    *pResponse.offset(1 as libc::c_int as isize) =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2c00
                                                                              as
                                                                              libc::c_uint)
               as *mut SDMMC_TypeDef)).RESP2;
    *pResponse.offset(2 as libc::c_int as isize) =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2c00
                                                                              as
                                                                              libc::c_uint)
               as *mut SDMMC_TypeDef)).RESP3;
    *pResponse.offset(3 as libc::c_int as isize) =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2c00
                                                                              as
                                                                              libc::c_uint)
               as *mut SDMMC_TypeDef)).RESP4;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  SD DMA transfer complete RX and TX callback.
  * @param  DMA_Stream_TypeDef* pDMA_Stream
  */
unsafe extern "C" fn SD_DMA_Complete(mut pDMA_Stream:
                                         *mut DMA_Stream_TypeDef) {
    if SD_Handle.RXCplt != 0 {
        if SD_Handle.Operation ==
               ((0 as libc::c_int) << 1 as libc::c_int |
                    SD_MULTIPLE_BLOCK as libc::c_int) as libc::c_uint {
            /* Send stop command in multiblock write */
            SD_TransmitCommand(12 as libc::c_int as uint8_t as libc::c_uint |
                                   (0x1 as libc::c_uint) << 6 as libc::c_uint,
                               0 as libc::c_int as uint32_t,
                               1 as libc::c_int as int8_t);
        }
        /* Disable the DMA transfer for transmit request by setting the DMAEN bit
	  	  in the SD DCTRL register */
        let ref mut fresh1 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut SDMMC_TypeDef)).DCTRL;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               3 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Clear all the static flags */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x10000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x2c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut SDMMC_TypeDef)).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) << 0 as libc::c_uint
                                        |
                                        (0x1 as libc::c_uint) <<
                                            1 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            2 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            3 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            4 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            6 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            7 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            8 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            10 as libc::c_uint);
        /* Clear flag */
        ::core::ptr::write_volatile(&mut SD_Handle.RXCplt as *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
        /* Disable the stream */
        ::core::ptr::write_volatile(&mut (*pDMA_Stream).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*pDMA_Stream).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    } else {
        /* Enable Dataend IE */
        let ref mut fresh2 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut SDMMC_TypeDef)).MASK;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             8 as libc::c_uint) as uint32_t as
                                        uint32_t)
    };
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Initializes all cards or single card as the case may be Card(s) come
  *         into standby state.
  * @retval SD Card error state
  */
unsafe extern "C" fn SD_InitializeCard() -> SD_Error_t {
    let mut ErrorState: SD_Error_t = SD_OK;
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x10000 as
                                              libc::c_uint).wrapping_add(0x2c00
                                                                             as
                                                                             libc::c_uint)
              as *mut SDMMC_TypeDef)).POWER &
           (0x3 as libc::c_uint) << 0 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        // Power off
        if SD_CardType as libc::c_uint !=
               SD_SECURE_DIGITAL_IO as libc::c_int as libc::c_uint {
            // Send CMD2 ALL_SEND_CID
            ErrorState =
                SD_TransmitCommand(2 as libc::c_int as uint8_t as libc::c_uint
                                       |
                                       (0x3 as libc::c_uint) <<
                                           6 as libc::c_uint,
                                   0 as libc::c_int as uint32_t,
                                   2 as libc::c_int as int8_t);
            if ErrorState as libc::c_uint !=
                   SD_OK as libc::c_int as libc::c_uint {
                return ErrorState
            }
            // Get Card identification number data
            SD_GetResponse(SD_Handle.CID.as_mut_ptr());
        }
        if SD_CardType as libc::c_uint ==
               SD_STD_CAPACITY_V1_1 as libc::c_int as libc::c_uint ||
               SD_CardType as libc::c_uint ==
                   SD_STD_CAPACITY_V2_0 as libc::c_int as libc::c_uint ||
               SD_CardType as libc::c_uint ==
                   SD_SECURE_DIGITAL_IO_COMBO as libc::c_int as libc::c_uint
               ||
               SD_CardType as libc::c_uint ==
                   SD_HIGH_CAPACITY as libc::c_int as libc::c_uint {
            // Send CMD3 SET_REL_ADDR with argument 0
            // SD Card publishes its RCA.
            ErrorState =
                SD_TransmitCommand(3 as libc::c_int as uint8_t as libc::c_uint
                                       |
                                       (0x1 as libc::c_uint) <<
                                           6 as libc::c_uint,
                                   0 as libc::c_int as uint32_t,
                                   6 as libc::c_int as int8_t);
            if ErrorState as libc::c_uint !=
                   SD_OK as libc::c_int as libc::c_uint {
                return ErrorState
            }
        }
        if SD_CardType as libc::c_uint !=
               SD_SECURE_DIGITAL_IO as libc::c_int as libc::c_uint {
            // Send CMD9 SEND_CSD with argument as card's RCA
            ErrorState =
                SD_TransmitCommand(9 as libc::c_int as uint8_t as libc::c_uint
                                       |
                                       (0x3 as libc::c_uint) <<
                                           6 as libc::c_uint, SD_CardRCA,
                                   2 as libc::c_int as int8_t);
            if ErrorState as libc::c_uint ==
                   SD_OK as libc::c_int as libc::c_uint {
                // Get Card Specific Data
                SD_GetResponse(SD_Handle.CSD.as_mut_ptr());
            }
        }
    } else { ErrorState = SD_REQUEST_NOT_APPLICABLE }
    return ErrorState;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Prepre the DMA transfer
  * @param  pDMA:         DMA Stream to use for the DMA operation
  * @param  pBuffer:      Pointer to the buffer that will contain the data to transmit
  * @param  BlockSize:    The SD card Data block size
  * @note   BlockSize must be 512 bytes.
  * @param  NumberOfBlocks: Number of blocks to write
  * @retval SD Card error state
  */
unsafe extern "C" fn SD_StartBlockTransfert(mut pBuffer: *mut uint32_t,
                                            mut BlockSize: uint32_t,
                                            mut NumberOfBlocks: uint32_t,
                                            mut dir: uint8_t) {
    let mut pDMA: *mut DMA_Stream_TypeDef =
        dma_stream; // Initialize data control register
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).DCTRL as
                                    *mut uint32_t,
                                0 as libc::c_int as
                                    uint32_t); // Initialize handle flags
    ::core::ptr::write_volatile(&mut SD_Handle.TransferComplete as
                                    *mut uint32_t,
                                0 as libc::c_int as
                                    uint32_t); // Initialize SD Read operation
    ::core::ptr::write_volatile(&mut SD_Handle.TransferError as *mut uint32_t,
                                SD_OK as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut SD_Handle.Operation as *mut uint32_t,
                                if NumberOfBlocks >
                                       1 as libc::c_int as libc::c_uint {
                                    SD_MULTIPLE_BLOCK as libc::c_int
                                } else { SD_SINGLE_BLOCK as libc::c_int } as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut SD_Handle.Operation as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&SD_Handle.Operation
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((dir as libc::c_int) <<
                                          1 as libc::c_int) as libc::c_uint)
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).MASK as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    if dir as libc::c_int == 0 as libc::c_int {
        let ref mut fresh3 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut SDMMC_TypeDef)).MASK;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              1 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  3 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  8 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  5 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh4 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut SDMMC_TypeDef)).MASK;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              1 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  3 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  4 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    if dir as libc::c_int == 1 as libc::c_int {
        let ref mut fresh5 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut SDMMC_TypeDef)).DCTRL;
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             3 as libc::c_uint) as uint32_t as
                                        uint32_t)
        // Enable SDMMC1 DMA transfer
    } // Disable the Peripheral
    ::core::ptr::write_volatile(&mut (*pDMA).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*pDMA).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t); // Configure DMA Stream data length
    while (*pDMA).CR & (0x1 as libc::c_uint) << 0 as libc::c_uint != 0 {
    } // Configure DMA Stream memory address
    ::core::ptr::write_volatile(&mut (*pDMA).NDTR as *mut uint32_t,
                                BlockSize.wrapping_mul(NumberOfBlocks).wrapping_div(4
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        libc::c_uint));
    ::core::ptr::write_volatile(&mut (*pDMA).M0AR as *mut uint32_t,
                                pBuffer as uint32_t);
    if dir as libc::c_int == 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*pDMA).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*pDMA).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t)
        // Sets peripheral to memory
    } else {
        ::core::ptr::write_volatile(&mut (*pDMA).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*pDMA).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t)
        // Sets memory to peripheral
    }
    if dma_stream ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x6400
                                                                              as
                                                                              libc::c_uint).wrapping_add(0x58
                                                                                                             as
                                                                                                             libc::c_uint)
               as *mut DMA_Stream_TypeDef {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef)).LIFCR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        25 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            24 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            22 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            26 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            27 as libc::c_uint)
        // Clear the transfer error flag
    } else {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef)).HIFCR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        19 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            18 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            16 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            20 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            21 as libc::c_uint)
        // Clear the transfer error flag
    } // Enable all interrupts
    ::core::ptr::write_volatile(&mut (*pDMA).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*pDMA).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          4 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              3 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              2 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              1 as libc::c_uint)) as uint32_t
                                    as uint32_t); // Enable the Peripheral
    ::core::ptr::write_volatile(&mut (*pDMA).FCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*pDMA).FCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*pDMA).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*pDMA).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    if dir as libc::c_int == 0 as libc::c_int {
        let ref mut fresh6 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut SDMMC_TypeDef)).DCTRL;
        ::core::ptr::write_volatile(fresh6,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             3 as libc::c_uint) as uint32_t as
                                        uint32_t)
        // Enable SDMMC1 DMA transfer
    };
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed by DMA mode.
  * @note   This API should be followed by the function SD_CheckOperation()
  *         to check the completion of the read process
  * @param  pReadBuffer: Pointer to the buffer that will contain the received data
  * @param  ReadAddr: Address from where data is to be read
  * @param  BlockSize: SD card Data block size
  * @note   BlockSize must be 512 bytes.
  * @param  NumberOfBlocks: Number of blocks to read.
  * @retval SD Card error state
  */
#[no_mangle]
pub unsafe extern "C" fn SD_ReadBlocks_DMA(mut ReadAddress: uint64_t,
                                           mut buffer: *mut uint32_t,
                                           mut BlockSize: uint32_t,
                                           mut NumberOfBlocks: uint32_t)
 -> SD_Error_t {
    let mut ErrorState: SD_Error_t = SD_OK;
    let mut CmdIndex: uint32_t = 0;
    ::core::ptr::write_volatile(&mut SD_Handle.RXCplt as *mut uint32_t,
                                1 as libc::c_int as uint32_t);
    //printf("Reading at %ld into %p %ld blocks\n", (uint32_t)ReadAddress, (void*)buffer, NumberOfBlocks);
    if SD_CardType as libc::c_uint !=
           SD_HIGH_CAPACITY as libc::c_int as libc::c_uint {
        ReadAddress =
            (ReadAddress as
                 libc::c_ulong).wrapping_mul(512 as libc::c_int as
                                                 libc::c_ulong) as uint64_t as
                uint64_t
    }
    SD_StartBlockTransfert(buffer, BlockSize, NumberOfBlocks,
                           0 as libc::c_int as uint8_t);
    // Configure the SD DPSM (Data Path State Machine)
    SD_DataTransferInit(BlockSize.wrapping_mul(NumberOfBlocks),
                        (0x1 as libc::c_uint) << 4 as libc::c_uint |
                            (0x8 as libc::c_uint) << 4 as libc::c_uint,
                        1 as libc::c_int != 0);
    // Set Block Size for Card
    ErrorState =
        SD_TransmitCommand(16 as libc::c_int as uint8_t as libc::c_uint |
                               (0x1 as libc::c_uint) << 6 as libc::c_uint,
                           BlockSize, 1 as libc::c_int as int8_t);
    // Send CMD18 READ_MULT_BLOCK with argument data address
    // or send CMD17 READ_SINGLE_BLOCK depending on number of block
    let mut retries: uint8_t = 10 as libc::c_int as uint8_t;
    CmdIndex =
        if NumberOfBlocks > 1 as libc::c_int as libc::c_uint {
            18 as libc::c_int as uint8_t as libc::c_int
        } else { 17 as libc::c_int as uint8_t as libc::c_int } as uint32_t;
    loop  {
        ErrorState =
            SD_TransmitCommand(CmdIndex |
                                   (0x1 as libc::c_uint) << 6 as libc::c_uint,
                               ReadAddress as uint32_t,
                               1 as libc::c_int as int8_t);
        if ErrorState as libc::c_uint != SD_OK as libc::c_int as libc::c_uint
               &&
               {
                   let fresh7 = retries;
                   retries = retries.wrapping_sub(1);
                   (fresh7 as libc::c_int) != 0
               } {
            ErrorState =
                SD_TransmitCommand(55 as libc::c_int as uint8_t as
                                       libc::c_uint |
                                       (0x1 as libc::c_uint) <<
                                           6 as libc::c_uint,
                                   0 as libc::c_int as uint32_t,
                                   1 as libc::c_int as int8_t)
        }
        if !(ErrorState as libc::c_uint !=
                 SD_OK as libc::c_int as libc::c_uint &&
                 retries as libc::c_int != 0) {
            break ;
        }
    }
    if ErrorState as libc::c_uint != SD_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut SD_Handle.RXCplt as *mut uint32_t,
                                    0 as libc::c_int as uint32_t)
    }
    // Update the SD transfer error in SD handle
    ::core::ptr::write_volatile(&mut SD_Handle.TransferError as *mut uint32_t,
                                ErrorState as uint32_t);
    return ErrorState;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Writes block(s) to a specified address in a card. The Data transfer
  *         is managed by DMA mode.
  * @note   This API should be followed by the function SD_CheckOperation()
  *         to check the completion of the write process (by SD current status polling).
  * @param  pWriteBuffer: pointer to the buffer that will contain the data to transmit
  * @param  WriteAddress: Address from where data is to be read
  * @param  BlockSize: the SD card Data block size
  * @note   BlockSize must be 512 bytes.
  * @param  NumberOfBlocks: Number of blocks to write
  * @retval SD Card error state
  */
#[no_mangle]
pub unsafe extern "C" fn SD_WriteBlocks_DMA(mut WriteAddress: uint64_t,
                                            mut buffer: *mut uint32_t,
                                            mut BlockSize: uint32_t,
                                            mut NumberOfBlocks: uint32_t)
 -> SD_Error_t {
    let mut ErrorState: SD_Error_t = SD_OK;
    let mut CmdIndex: uint32_t = 0;
    ::core::ptr::write_volatile(&mut SD_Handle.TXCplt as *mut uint32_t,
                                1 as libc::c_int as uint32_t);
    //printf("Reading at %ld into %p %ld blocks\n", (uint32_t)WriteAddress, (void*)buffer, NumberOfBlocks);
    if SD_CardType as libc::c_uint !=
           SD_HIGH_CAPACITY as libc::c_int as libc::c_uint {
        WriteAddress =
            (WriteAddress as
                 libc::c_ulong).wrapping_mul(512 as libc::c_int as
                                                 libc::c_ulong) as uint64_t as
                uint64_t
    }
    // Check number of blocks command
    // Send CMD24 WRITE_SINGLE_BLOCK
    // Send CMD25 WRITE_MULT_BLOCK with argument data address
    CmdIndex =
        if NumberOfBlocks > 1 as libc::c_int as libc::c_uint {
            25 as libc::c_int as uint8_t as libc::c_int
        } else { 24 as libc::c_int as uint8_t as libc::c_int } as uint32_t;
    // Set Block Size for Card
    let mut retries: uint8_t = 10 as libc::c_int as uint8_t;
    loop  {
        ErrorState =
            SD_TransmitCommand(CmdIndex |
                                   (0x1 as libc::c_uint) << 6 as libc::c_uint,
                               WriteAddress as uint32_t,
                               1 as libc::c_int as int8_t);
        if ErrorState as libc::c_uint != SD_OK as libc::c_int as libc::c_uint
               &&
               {
                   let fresh8 = retries;
                   retries = retries.wrapping_sub(1);
                   (fresh8 as libc::c_int) != 0
               } {
            ErrorState =
                SD_TransmitCommand(55 as libc::c_int as uint8_t as
                                       libc::c_uint |
                                       (0x1 as libc::c_uint) <<
                                           6 as libc::c_uint,
                                   0 as libc::c_int as uint32_t,
                                   1 as libc::c_int as int8_t)
        }
        if !(ErrorState as libc::c_uint !=
                 SD_OK as libc::c_int as libc::c_uint &&
                 retries as libc::c_int != 0) {
            break ;
        }
    }
    if ErrorState as libc::c_uint != SD_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut SD_Handle.TXCplt as *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
        return ErrorState
    }
    SD_StartBlockTransfert(buffer, BlockSize, NumberOfBlocks,
                           1 as libc::c_int as uint8_t);
    // Configure the SD DPSM (Data Path State Machine)
    SD_DataTransferInit(BlockSize.wrapping_mul(NumberOfBlocks),
                        (0x1 as libc::c_uint) << 4 as libc::c_uint |
                            (0x8 as libc::c_uint) << 4 as libc::c_uint,
                        0 as libc::c_int != 0);
    ::core::ptr::write_volatile(&mut SD_Handle.TransferError as *mut uint32_t,
                                ErrorState as uint32_t);
    return ErrorState;
}
#[no_mangle]
pub unsafe extern "C" fn SD_CheckWrite() -> SD_Error_t {
    if SD_Handle.TXCplt != 0 as libc::c_int as libc::c_uint { return SD_BUSY }
    return SD_OK;
}
#[no_mangle]
pub unsafe extern "C" fn SD_CheckRead() -> SD_Error_t {
    if SD_Handle.RXCplt != 0 as libc::c_int as libc::c_uint { return SD_BUSY }
    return SD_OK;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  This function waits until the SD DMA data wirte or read transfer is finished.
  *         This should be called after WriteBlocks_DMA or SD_ReadBlocks_DMA() function
  *         to insure that all data sent is already transferred by the DMA controller.
  * @retval SD Card error state
  */
/*
SD_Error_t SD_CheckOperation(uint32_t Flag)
{
    SD_Error_t  ErrorState = SD_OK;
    uint32_t    TimeOut;
    uint32_t    Temp1;
    uint32_t    Temp2;
    SD_Error_t  Temp3;

    // Wait for DMA/SD transfer end or SD error variables to be in SD handle
    Temp1 = SD_Handle.DMA_XferComplete;
    Temp2 = SD_Handle.TransferComplete;
    Temp3 = (SD_Error_t)SD_Handle.TransferError;

    if (((Temp1 & Temp2) == 0) && (Temp3 == SD_OK) && (TimeOut > 0))
    {
        Temp1 = SD_Handle.DMA_XferComplete;
        Temp2 = SD_Handle.TransferComplete;
        Temp3 = (SD_Error_t)SD_Handle.TransferError;
        TimeOut--;
        return SD_BUSY;
    }

    // Wait until the Rx transfer is no longer active
    if (((SDMMC1->STA & Flag) != 0) && (TimeOut > 0))
    {
        TimeOut--;
        return SD_BUSY;
    }

    // Send stop command in multi block read
    if(SD_Handle.Operation & 0x01 == SD_MULTIPLE_BLOCK)
    {
        ErrorState = SD_TransmitCommand((SD_CMD_STOP_TRANSMISSION | SD_CMD_RESPONSE_SHORT), 0, 1);
    }

    if((TimeOut == 0) && (ErrorState == SD_OK))
    {
        ErrorState = SD_DATA_TIMEOUT;
    }

    // Return error state
    if(SD_Handle.TransferError != SD_OK)
    {
        return (SD_Error_t)(SD_Handle.TransferError);
    }

    return ErrorState;
}
*/
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Erases the specified memory area of the given SD card.
  * @param  StartAddress: Start byte address
  * @param  EndAddress: End byte address
  * @retval SD Card error state
  */
/*
SD_Error_t SD_Erase(uint64_t StartAddress, uint64_t EndAddress)
{
    SD_Error_t ErrorState;
    uint32_t   Delay;
    uint32_t   MaxDelay;
    uint8_t    CardState;

    // Check if the card command class supports erase command
    if(((SD_Handle.CSD[1] >> 20) & SD_CCCC_ERASE) == 0)
    {
        return SD_REQUEST_NOT_APPLICABLE;
    }

    // Get max delay value
    MaxDelay = 120000 / (((SDMMC1->CLKCR) & 0xFF) + 2);

    if((SDMMC1->RESP1 & SD_CARD_LOCKED) == SD_CARD_LOCKED)
    {
        return SD_LOCK_UNLOCK_FAILED;
    }

    // Get start and end block for high capacity cards
    if(SD_CardType == SD_HIGH_CAPACITY)
    {
        StartAddress /= 512;
        EndAddress   /= 512;
    }

    // According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33)
    if ((SD_CardType == SD_STD_CAPACITY_V1_1) || (SD_CardType == SD_STD_CAPACITY_V2_0) ||
        (SD_CardType == SD_HIGH_CAPACITY))
    {
        // Send CMD32 SD_ERASE_GRP_START with argument as addr
        if((ErrorState = SD_TransmitCommand((SD_CMD_SD_ERASE_GRP_START | SDMMC_CMD_RESPONSE_SHORT), (uint32_t)StartAddress, 1)) != SD_OK)
        {
            return ErrorState;
        }

        // Send CMD33 SD_ERASE_GRP_END with argument as addr
        if((ErrorState = SD_TransmitCommand((SD_CMD_SD_ERASE_GRP_END | SDMMC_CMD_RESPONSE_SHORT), (uint32_t)EndAddress, 1)) != SD_OK)
        {
            return ErrorState;
        }
    }

    // Send CMD38 ERASE
    if((ErrorState = SD_TransmitCommand((SD_CMD_ERASE | SDMMC_CMD_RESPONSE_SHORT), 0, 1)) != SD_OK)
    {
        return ErrorState;
    }

    for(Delay = 0; Delay < MaxDelay; Delay++);

    // Wait until the card is in programming state
    ErrorState = SD_IsCardProgramming(&CardState);

    Delay = SD_DATATIMEOUT;
    while((Delay > 0) && (ErrorState == SD_OK) && ((CardState == SD_CARD_PROGRAMMING) || (CardState == SD_CARD_RECEIVING)))
    {
        ErrorState = SD_IsCardProgramming( &CardState);
        Delay--;
    }

    return ErrorState;
}
*/
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Returns information about specific card.
  *         contains all SD cardinformation
  * @retval SD Card error state
  */
#[no_mangle]
pub unsafe extern "C" fn SD_GetCardInfo() -> SD_Error_t {
    let mut ErrorState: SD_Error_t = SD_OK;
    let mut Temp: uint32_t = 0 as libc::c_int as uint32_t;
    // Byte 0
    Temp =
        (SD_Handle.CSD[0 as libc::c_int as usize] &
             0xff000000 as libc::c_uint) >> 24 as libc::c_int;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.CSDStruct as
                                    *mut uint8_t,
                                ((Temp & 0xc0 as libc::c_int as libc::c_uint)
                                     >> 6 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.SysSpecVersion as
                                    *mut uint8_t,
                                ((Temp & 0x3c as libc::c_int as libc::c_uint)
                                     >> 2 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.Reserved1 as
                                    *mut uint8_t,
                                (Temp & 0x3 as libc::c_int as libc::c_uint) as
                                    uint8_t);
    // Byte 1
    Temp =
        (SD_Handle.CSD[0 as libc::c_int as usize] &
             0xff0000 as libc::c_int as libc::c_uint) >> 16 as libc::c_int;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.TAAC as *mut uint8_t,
                                Temp as uint8_t);
    // Byte 2
    Temp =
        (SD_Handle.CSD[0 as libc::c_int as usize] &
             0xff00 as libc::c_int as libc::c_uint) >> 8 as libc::c_int;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.NSAC as *mut uint8_t,
                                Temp as uint8_t);
    // Byte 3
    Temp =
        SD_Handle.CSD[0 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.MaxBusClkFrec as
                                    *mut uint8_t, Temp as uint8_t);
    // Byte 4
    Temp =
        (SD_Handle.CSD[1 as libc::c_int as usize] &
             0xff000000 as libc::c_uint) >> 24 as libc::c_int;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.CardComdClasses as
                                    *mut uint16_t,
                                (Temp << 4 as libc::c_int) as uint16_t);
    // Byte 5
    Temp =
        (SD_Handle.CSD[1 as libc::c_int as usize] &
             0xff0000 as libc::c_int as libc::c_uint) >> 16 as libc::c_int;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.CardComdClasses as
                                    *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&SD_CardInfo.SD_csd.CardComdClasses
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     ((Temp &
                                           0xf0 as libc::c_int as
                                               libc::c_uint) >>
                                          4 as libc::c_int) as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.RdBlockLen as
                                    *mut uint8_t,
                                (Temp & 0xf as libc::c_int as libc::c_uint) as
                                    uint8_t);
    // Byte 6
    Temp =
        (SD_Handle.CSD[1 as libc::c_int as usize] &
             0xff00 as libc::c_int as libc::c_uint) >>
            8 as libc::c_int; /* !< Reserved */
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.PartBlockRead as
                                    *mut uint8_t,
                                ((Temp & 0x80 as libc::c_int as libc::c_uint)
                                     >> 7 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.WrBlockMisalign as
                                    *mut uint8_t,
                                ((Temp & 0x40 as libc::c_int as libc::c_uint)
                                     >> 6 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.RdBlockMisalign as
                                    *mut uint8_t,
                                ((Temp & 0x20 as libc::c_int as libc::c_uint)
                                     >> 5 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.DSRImpl as
                                    *mut uint8_t,
                                ((Temp & 0x10 as libc::c_int as libc::c_uint)
                                     >> 4 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.Reserved2 as
                                    *mut uint8_t,
                                0 as libc::c_int as uint8_t);
    if SD_CardType as libc::c_uint ==
           SD_STD_CAPACITY_V1_1 as libc::c_int as libc::c_uint ||
           SD_CardType as libc::c_uint ==
               SD_STD_CAPACITY_V2_0 as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.DeviceSize as
                                        *mut uint32_t,
                                    (Temp &
                                         0x3 as libc::c_int as libc::c_uint)
                                        << 10 as libc::c_int);
        // Byte 7
        Temp =
            (SD_Handle.CSD[1 as libc::c_int as usize] &
                 0xff as libc::c_int as libc::c_uint) as uint8_t as uint32_t;
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.DeviceSize as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&SD_CardInfo.SD_csd.DeviceSize
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         Temp << 2 as libc::c_int) as uint32_t
                                        as uint32_t);
        // Byte 8
        Temp =
            ((SD_Handle.CSD[2 as libc::c_int as usize] &
                  0xff000000 as libc::c_uint) >> 24 as libc::c_int) as uint8_t
                as uint32_t;
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.DeviceSize as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&SD_CardInfo.SD_csd.DeviceSize
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (Temp &
                                              0xc0 as libc::c_int as
                                                  libc::c_uint) >>
                                             6 as libc::c_int) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.MaxRdCurrentVDDMin
                                        as *mut uint8_t,
                                    ((Temp &
                                          0x38 as libc::c_int as libc::c_uint)
                                         >> 3 as libc::c_int) as uint8_t);
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.MaxRdCurrentVDDMax
                                        as *mut uint8_t,
                                    (Temp &
                                         0x7 as libc::c_int as libc::c_uint)
                                        as uint8_t);
        // Byte 9
        Temp =
            ((SD_Handle.CSD[2 as libc::c_int as usize] &
                  0xff0000 as libc::c_int as libc::c_uint) >>
                 16 as libc::c_int) as uint8_t as uint32_t;
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.MaxWrCurrentVDDMin
                                        as *mut uint8_t,
                                    ((Temp &
                                          0xe0 as libc::c_int as libc::c_uint)
                                         >> 5 as libc::c_int) as uint8_t);
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.MaxWrCurrentVDDMax
                                        as *mut uint8_t,
                                    ((Temp &
                                          0x1c as libc::c_int as libc::c_uint)
                                         >> 2 as libc::c_int) as uint8_t);
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.DeviceSizeMul as
                                        *mut uint8_t,
                                    ((Temp &
                                          0x3 as libc::c_int as libc::c_uint)
                                         << 1 as libc::c_int) as uint8_t);
        // Byte 10
        Temp =
            ((SD_Handle.CSD[2 as libc::c_int as usize] &
                  0xff00 as libc::c_int as libc::c_uint) >> 8 as libc::c_int)
                as uint8_t as uint32_t;
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.DeviceSizeMul as
                                        *mut uint8_t,
                                    (::core::ptr::read_volatile::<uint8_t>(&SD_CardInfo.SD_csd.DeviceSizeMul
                                                                               as
                                                                               *const uint8_t)
                                         as libc::c_uint |
                                         (Temp &
                                              0x80 as libc::c_int as
                                                  libc::c_uint) >>
                                             7 as libc::c_int) as uint8_t as
                                        uint8_t);
        SD_CardInfo.CardCapacity =
            SD_CardInfo.SD_csd.DeviceSize.wrapping_add(1 as libc::c_int as
                                                           libc::c_uint) as
                uint64_t;
        SD_CardInfo.CardCapacity =
            (SD_CardInfo.CardCapacity as
                 libc::c_ulong).wrapping_mul(((1 as libc::c_int) <<
                                                  SD_CardInfo.SD_csd.DeviceSizeMul
                                                      as libc::c_int +
                                                      2 as libc::c_int) as
                                                 libc::c_ulong) as uint64_t as
                uint64_t;
        SD_CardInfo.CardBlockSize =
            ((1 as libc::c_int) <<
                 SD_CardInfo.SD_csd.RdBlockLen as libc::c_int) as uint32_t;
        SD_CardInfo.CardCapacity =
            (SD_CardInfo.CardCapacity as
                 libc::c_ulong).wrapping_mul(SD_CardInfo.CardBlockSize as
                                                 libc::c_ulong) as uint64_t as
                uint64_t
    } else if SD_CardType as libc::c_uint ==
                  SD_HIGH_CAPACITY as libc::c_int as libc::c_uint {
        // Byte 7
        Temp =
            (SD_Handle.CSD[1 as libc::c_int as usize] &
                 0xff as libc::c_int as libc::c_uint) as uint8_t as uint32_t;
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.DeviceSize as
                                        *mut uint32_t,
                                    (Temp &
                                         0x3f as libc::c_int as libc::c_uint)
                                        << 16 as libc::c_int);
        // Byte 8
        Temp =
            ((SD_Handle.CSD[2 as libc::c_int as usize] &
                  0xff000000 as libc::c_uint) >> 24 as libc::c_int) as uint8_t
                as uint32_t;
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.DeviceSize as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&SD_CardInfo.SD_csd.DeviceSize
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         Temp << 8 as libc::c_int) as uint32_t
                                        as uint32_t);
        // Byte 9
        Temp =
            ((SD_Handle.CSD[2 as libc::c_int as usize] &
                  0xff0000 as libc::c_int as libc::c_uint) >>
                 16 as libc::c_int) as uint8_t as uint32_t;
        ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.DeviceSize as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&SD_CardInfo.SD_csd.DeviceSize
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | Temp) as uint32_t
                                        as uint32_t);
        // Byte 10
        Temp =
            ((SD_Handle.CSD[2 as libc::c_int as usize] &
                  0xff00 as libc::c_int as libc::c_uint) >> 8 as libc::c_int)
                as uint8_t as uint32_t;
        SD_CardInfo.CardCapacity =
            (SD_CardInfo.SD_csd.DeviceSize as
                 uint64_t).wrapping_add(1 as libc::c_int as
                                            libc::c_ulong).wrapping_mul(1024
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_ulong);
        SD_CardInfo.CardBlockSize = 512 as libc::c_int as uint32_t
    } else {
        // Not supported card type
        ErrorState = SD_ERROR
    }
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.EraseGrSize as
                                    *mut uint8_t,
                                ((Temp & 0x40 as libc::c_int as libc::c_uint)
                                     >> 6 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.EraseGrMul as
                                    *mut uint8_t,
                                ((Temp & 0x3f as libc::c_int as libc::c_uint)
                                     << 1 as libc::c_int) as uint8_t);
    // Byte 11
    Temp =
        (SD_Handle.CSD[2 as libc::c_int as usize] &
             0xff as libc::c_int as libc::c_uint) as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.EraseGrMul as
                                    *mut uint8_t,
                                (::core::ptr::read_volatile::<uint8_t>(&SD_CardInfo.SD_csd.EraseGrMul
                                                                           as
                                                                           *const uint8_t)
                                     as libc::c_uint |
                                     (Temp &
                                          0x80 as libc::c_int as libc::c_uint)
                                         >> 7 as libc::c_int) as uint8_t as
                                    uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.WrProtectGrSize as
                                    *mut uint8_t,
                                (Temp & 0x7f as libc::c_int as libc::c_uint)
                                    as uint8_t);
    // Byte 12
    Temp =
        ((SD_Handle.CSD[3 as libc::c_int as usize] &
              0xff000000 as libc::c_uint) >> 24 as libc::c_int) as uint8_t as
            uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.WrProtectGrEnable as
                                    *mut uint8_t,
                                ((Temp & 0x80 as libc::c_int as libc::c_uint)
                                     >> 7 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.ManDeflECC as
                                    *mut uint8_t,
                                ((Temp & 0x60 as libc::c_int as libc::c_uint)
                                     >> 5 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.WrSpeedFact as
                                    *mut uint8_t,
                                ((Temp & 0x1c as libc::c_int as libc::c_uint)
                                     >> 2 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.MaxWrBlockLen as
                                    *mut uint8_t,
                                ((Temp & 0x3 as libc::c_int as libc::c_uint)
                                     << 2 as libc::c_int) as uint8_t);
    // Byte 13
    Temp =
        ((SD_Handle.CSD[3 as libc::c_int as usize] &
              0xff0000 as libc::c_int as libc::c_uint) >> 16 as libc::c_int)
            as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.MaxWrBlockLen as
                                    *mut uint8_t,
                                (::core::ptr::read_volatile::<uint8_t>(&SD_CardInfo.SD_csd.MaxWrBlockLen
                                                                           as
                                                                           *const uint8_t)
                                     as libc::c_uint |
                                     (Temp &
                                          0xc0 as libc::c_int as libc::c_uint)
                                         >> 6 as libc::c_int) as uint8_t as
                                    uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.WriteBlockPaPartial as
                                    *mut uint8_t,
                                ((Temp & 0x20 as libc::c_int as libc::c_uint)
                                     >> 5 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.Reserved3 as
                                    *mut uint8_t,
                                0 as libc::c_int as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.ContentProtectAppli as
                                    *mut uint8_t,
                                (Temp & 0x1 as libc::c_int as libc::c_uint) as
                                    uint8_t);
    // Byte 14
    Temp =
        ((SD_Handle.CSD[3 as libc::c_int as usize] &
              0xff00 as libc::c_int as libc::c_uint) >> 8 as libc::c_int) as
            uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.FileFormatGrouop as
                                    *mut uint8_t,
                                ((Temp & 0x80 as libc::c_int as libc::c_uint)
                                     >> 7 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.CopyFlag as
                                    *mut uint8_t,
                                ((Temp & 0x40 as libc::c_int as libc::c_uint)
                                     >> 6 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.PermWrProtect as
                                    *mut uint8_t,
                                ((Temp & 0x20 as libc::c_int as libc::c_uint)
                                     >> 5 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.TempWrProtect as
                                    *mut uint8_t,
                                ((Temp & 0x10 as libc::c_int as libc::c_uint)
                                     >> 4 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.FileFormat as
                                    *mut uint8_t,
                                ((Temp & 0xc as libc::c_int as libc::c_uint)
                                     >> 2 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.ECC as *mut uint8_t,
                                (Temp & 0x3 as libc::c_int as libc::c_uint) as
                                    uint8_t);
    // Byte 15
    Temp =
        (SD_Handle.CSD[3 as libc::c_int as usize] &
             0xff as libc::c_int as libc::c_uint) as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.CSD_CRC as
                                    *mut uint8_t,
                                ((Temp & 0xfe as libc::c_int as libc::c_uint)
                                     >> 1 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_csd.Reserved4 as
                                    *mut uint8_t,
                                1 as libc::c_int as uint8_t);
    // Byte 0
    Temp =
        ((SD_Handle.CID[0 as libc::c_int as usize] &
              0xff000000 as libc::c_uint) >> 24 as libc::c_int) as uint8_t as
            uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ManufacturerID as
                                    *mut uint8_t, Temp as uint8_t);
    // Byte 1
    Temp =
        ((SD_Handle.CID[0 as libc::c_int as usize] &
              0xff0000 as libc::c_int as libc::c_uint) >> 16 as libc::c_int)
            as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.OEM_AppliID as
                                    *mut uint16_t,
                                (Temp << 8 as libc::c_int) as uint16_t);
    // Byte 2
    Temp =
        ((SD_Handle.CID[0 as libc::c_int as usize] &
              0xff00 as libc::c_int as libc::c_uint) >> 8 as libc::c_int) as
            uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.OEM_AppliID as
                                    *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&SD_CardInfo.SD_cid.OEM_AppliID
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_uint | Temp) as uint16_t as
                                    uint16_t);
    // Byte 3
    Temp =
        (SD_Handle.CID[0 as libc::c_int as usize] &
             0xff as libc::c_int as libc::c_uint) as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ProdName1 as
                                    *mut uint32_t, Temp << 24 as libc::c_int);
    // Byte 4
    Temp =
        ((SD_Handle.CID[1 as libc::c_int as usize] &
              0xff000000 as libc::c_uint) >> 24 as libc::c_int) as uint8_t as
            uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ProdName1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&SD_CardInfo.SD_cid.ProdName1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     Temp << 16 as libc::c_int) as uint32_t as
                                    uint32_t);
    // Byte 5
    Temp =
        ((SD_Handle.CID[1 as libc::c_int as usize] &
              0xff0000 as libc::c_int as libc::c_uint) >> 16 as libc::c_int)
            as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ProdName1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&SD_CardInfo.SD_cid.ProdName1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     Temp << 8 as libc::c_int) as uint32_t as
                                    uint32_t);
    // Byte 6
    Temp =
        ((SD_Handle.CID[1 as libc::c_int as usize] &
              0xff00 as libc::c_int as libc::c_uint) >> 8 as libc::c_int) as
            uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ProdName1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&SD_CardInfo.SD_cid.ProdName1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Temp) as uint32_t as
                                    uint32_t);
    // Byte 7
    Temp =
        (SD_Handle.CID[1 as libc::c_int as usize] &
             0xff as libc::c_int as libc::c_uint) as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ProdName2 as
                                    *mut uint8_t, Temp as uint8_t);
    // Byte 8
    Temp =
        ((SD_Handle.CID[2 as libc::c_int as usize] &
              0xff000000 as libc::c_uint) >> 24 as libc::c_int) as uint8_t as
            uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ProdRev as
                                    *mut uint8_t, Temp as uint8_t);
    // Byte 9
    Temp =
        ((SD_Handle.CID[2 as libc::c_int as usize] &
              0xff0000 as libc::c_int as libc::c_uint) >> 16 as libc::c_int)
            as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ProdSN as
                                    *mut uint32_t, Temp << 24 as libc::c_int);
    // Byte 10
    Temp =
        ((SD_Handle.CID[2 as libc::c_int as usize] &
              0xff00 as libc::c_int as libc::c_uint) >> 8 as libc::c_int) as
            uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ProdSN as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&SD_CardInfo.SD_cid.ProdSN
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     Temp << 16 as libc::c_int) as uint32_t as
                                    uint32_t);
    // Byte 11
    Temp =
        (SD_Handle.CID[2 as libc::c_int as usize] &
             0xff as libc::c_int as libc::c_uint) as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ProdSN as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&SD_CardInfo.SD_cid.ProdSN
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     Temp << 8 as libc::c_int) as uint32_t as
                                    uint32_t);
    // Byte 12
    Temp =
        ((SD_Handle.CID[3 as libc::c_int as usize] &
              0xff000000 as libc::c_uint) >> 24 as libc::c_int) as uint8_t as
            uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ProdSN as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&SD_CardInfo.SD_cid.ProdSN
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Temp) as uint32_t as
                                    uint32_t);
    // Byte 13
    Temp =
        ((SD_Handle.CID[3 as libc::c_int as usize] &
              0xff0000 as libc::c_int as libc::c_uint) >> 16 as libc::c_int)
            as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.Reserved1 as
                                    *mut uint8_t,
                                (::core::ptr::read_volatile::<uint8_t>(&SD_CardInfo.SD_cid.Reserved1
                                                                           as
                                                                           *const uint8_t)
                                     as libc::c_uint |
                                     (Temp &
                                          0xf0 as libc::c_int as libc::c_uint)
                                         >> 4 as libc::c_int) as uint8_t as
                                    uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ManufactDate as
                                    *mut uint16_t,
                                ((Temp & 0xf as libc::c_int as libc::c_uint)
                                     << 8 as libc::c_int) as uint16_t);
    // Byte 14
    Temp =
        ((SD_Handle.CID[3 as libc::c_int as usize] &
              0xff00 as libc::c_int as libc::c_uint) >> 8 as libc::c_int) as
            uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.ManufactDate as
                                    *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&SD_CardInfo.SD_cid.ManufactDate
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_uint | Temp) as uint16_t as
                                    uint16_t);
    // Byte 15
    Temp =
        (SD_Handle.CID[3 as libc::c_int as usize] &
             0xff as libc::c_int as libc::c_uint) as uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.CID_CRC as
                                    *mut uint8_t,
                                ((Temp & 0xfe as libc::c_int as libc::c_uint)
                                     >> 1 as libc::c_int) as uint8_t);
    ::core::ptr::write_volatile(&mut SD_CardInfo.SD_cid.Reserved2 as
                                    *mut uint8_t,
                                1 as libc::c_int as uint8_t);
    return ErrorState;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Enables wide bus operation for the requested card if supported by
  *         card.
  * @param  WideMode: Specifies the SD card wide bus mode
  *          This parameter can be one of the following values:
  *            @arg SD_BUS_WIDE_8B: 8-bit data transfer (Only for MMC)
  *            @arg SD_BUS_WIDE_4B: 4-bit data transfer
  *            @arg SD_BUS_WIDE_1B: 1-bit data transfer
  * @retval SD Card error state
  */
unsafe extern "C" fn SD_WideBusOperationConfig(mut WideMode: uint32_t)
 -> SD_Error_t {
    let mut ErrorState: SD_Error_t = SD_OK;
    let mut Temp: uint32_t = 0;
    let mut SCR: [uint32_t; 2] =
        [0 as libc::c_int as uint32_t, 0 as libc::c_int as uint32_t];
    if SD_CardType as libc::c_uint ==
           SD_STD_CAPACITY_V1_1 as libc::c_int as libc::c_uint ||
           SD_CardType as libc::c_uint ==
               SD_STD_CAPACITY_V2_0 as libc::c_int as libc::c_uint ||
           SD_CardType as libc::c_uint ==
               SD_HIGH_CAPACITY as libc::c_int as libc::c_uint {
        if WideMode == (0x2 as libc::c_uint) << 11 as libc::c_uint {
            ErrorState = SD_UNSUPPORTED_FEATURE
        } else if WideMode == (0x1 as libc::c_uint) << 11 as libc::c_uint ||
                      WideMode == 0 as libc::c_int as uint32_t {
            if (*((0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0x2c00
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut SDMMC_TypeDef)).RESP1 &
                   0x2000000 as libc::c_int as uint32_t !=
                   0x2000000 as libc::c_int as uint32_t {
                // Get SCR Register
                ErrorState = SD_FindSCR(SCR.as_mut_ptr());
                if ErrorState as libc::c_uint ==
                       SD_OK as libc::c_int as libc::c_uint {
                    Temp =
                        if WideMode ==
                               (0x1 as libc::c_uint) << 11 as libc::c_uint {
                            0x40000 as libc::c_int as uint32_t
                        } else { 0x10000 as libc::c_int as uint32_t };
                    // If requested card supports wide bus operation
                    if SCR[1 as libc::c_int as usize] & Temp !=
                           0 as libc::c_int as uint32_t {
                        // Send CMD55 APP_CMD with argument as card's RCA.
                        ErrorState =
                            SD_TransmitCommand(55 as libc::c_int as uint8_t as
                                                   libc::c_uint |
                                                   (0x1 as libc::c_uint) <<
                                                       6 as libc::c_uint,
                                               SD_CardRCA,
                                               1 as libc::c_int as int8_t);
                        if ErrorState as libc::c_uint ==
                               SD_OK as libc::c_int as libc::c_uint {
                            Temp =
                                if WideMode ==
                                       (0x1 as libc::c_uint) <<
                                           11 as libc::c_uint {
                                    2 as libc::c_int
                                } else { 0 as libc::c_int } as uint32_t;
                            // Send ACMD6 APP_CMD with argument as 2 for wide bus mode
                            ErrorState =
                                SD_TransmitCommand(6 as libc::c_int as uint8_t
                                                       as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           6 as libc::c_uint,
                                                   Temp,
                                                   1 as libc::c_int as int8_t)
                        }
                    } else { ErrorState = SD_REQUEST_NOT_APPLICABLE }
                }
            } else { ErrorState = SD_LOCK_UNLOCK_FAILED }
        } else {
            ErrorState = SD_INVALID_PARAMETER
            // WideMode is not a valid argument
        }
        if ErrorState as libc::c_uint == SD_OK as libc::c_int as libc::c_uint
           {
            // Configure the SDMMC1 peripheral, we need this delay for some reason...
            while (*((0x40000000 as
                          libc::c_uint).wrapping_add(0x10000 as
                                                         libc::c_uint).wrapping_add(0x2c00
                                                                                        as
                                                                                        libc::c_uint)
                         as *mut SDMMC_TypeDef)).CLKCR &
                      0x800 as libc::c_int as libc::c_uint != WideMode {
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x10000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x2c00
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut SDMMC_TypeDef)).CLKCR
                                                as *mut uint32_t,
                                            (*((0x40000000 as
                                                    libc::c_uint).wrapping_add(0x10000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                                                  as
                                                                                                                  libc::c_uint)
                                                   as
                                                   *mut SDMMC_TypeDef)).CLKCR
                                                &
                                                !((0xff as libc::c_uint) <<
                                                      0 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          9 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          10 as libc::c_uint |
                                                      (0x3 as libc::c_uint) <<
                                                          11 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          13 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          14 as libc::c_uint)
                                                | WideMode)
            }
        }
    } else { ErrorState = SD_UNSUPPORTED_FEATURE }
    return ErrorState;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Switches the SD card to High Speed mode.
  *         This API must be used after "Transfer State"
  * @retval SD Card error state
  */
/*
SD_Error_t HAL_SD_HighSpeed(void)
{
    SD_Error_t  ErrorState;
    uint8_t     SD_hs[64]  = {0};
    uint32_t    SD_scr[2]  = {0, 0};
    uint32_t    SD_SPEC    = 0;
    uint32_t    Count      = 0;
    uint32_t*   Buffer     = (uint32_t *)SD_hs;

    // Initialize the Data control register
    SDMMC1->DCTRL = 0;

    // Get SCR Register
    if((ErrorState = SD_FindSCR(SD_scr)) != SD_OK)
    {
        return ErrorState;
    }

    // Test the Version supported by the card
    SD_SPEC = (SD_scr[1]  & 0x01000000) | (SD_scr[1]  & 0x02000000);

    if(SD_SPEC != SD_ALLZERO)
    {
        // Set Block Size for Card
        if((ErrorState = SD_TransmitCommand((SD_CMD_SET_BLOCKLEN | SDMMC_CMD_RESPONSE_SHORT), 64, 1)) != SD_OK)
        {
            return ErrorState;
        }

        // Configure the SD DPSM (Data Path State Machine)
        SD_DataTransferInit(64, SDMMC_DATABLOCK_SIZE_64B, true);

        // Send CMD6 switch mode
        if((ErrorState =SD_TransmitCommand((SD_CMD_HS_SWITCH | SDMMC_CMD_RESPONSE_SHORT), 0x80FFFF01, 1)) != SD_OK)
        {
            return ErrorState;
        }

        while((SDMMC1->STA & (SDMMC_STA_RXOVERR | SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT | SDMMC_STA_DBCKEND)) == 0)
        {
            if((SDMMC1->STA & SDMMC_STA_RXFIFOHF) != 0)
            {
                for(Count = 0; Count < 8; Count++)
                {
                    *(Buffer + Count) = SDMMC1->FIFO;
                }

                Buffer += 8;
            }
        }

        if((SDMMC1->STA & SDMMC_STA_DTIMEOUT) != 0)        return SD_DATA_TIMEOUT;
        else if((SDMMC1->STA & SDMMC_STA_DCRCFAIL) != 0)   return SD_DATA_CRC_FAIL;
        else if((SDMMC1->STA & SDMMC_STA_RXOVERR) != 0)    return SD_RX_OVERRUN;

        Count = SD_DATATIMEOUT;

        while(((SDMMC1->STA & SDMMC_STA_RXDAVL) != 0) && (Count > 0))
        {
            *Buffer = SDMMC1->FIFO;
            Buffer++;
            Count--;
        }

        // Test if the switch mode HS is ok
        if((SD_hs[13] & 2) != 2)
        {
            ErrorState = SD_UNSUPPORTED_FEATURE;
        }
    }

    return ErrorState;
}

*/
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Gets the current card's data status.
  * @retval Data Transfer state
  */
#[no_mangle]
pub unsafe extern "C" fn SD_GetStatus() -> SD_Error_t {
    let mut ErrorState: SD_Error_t = SD_OK;
    let mut Response1: uint32_t = 0;
    let mut CardState: SD_CardState_t = 0 as SD_CardState_t;
    // Send Status command
    ErrorState =
        SD_TransmitCommand(13 as libc::c_int as uint8_t as libc::c_uint |
                               (0x1 as libc::c_uint) << 6 as libc::c_uint,
                           SD_CardRCA, 1 as libc::c_int as int8_t);
    if ErrorState as libc::c_uint == SD_OK as libc::c_int as libc::c_uint {
        Response1 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut SDMMC_TypeDef)).RESP1;
        CardState =
            (Response1 >> 9 as libc::c_int &
                 0xf as libc::c_int as libc::c_uint) as SD_CardState_t;
        // Find SD status according to card state
        if CardState as libc::c_uint ==
               SD_CARD_TRANSFER as libc::c_int as libc::c_uint {
            ErrorState = SD_OK
        } else if CardState as libc::c_uint ==
                      SD_CARD_ERROR as libc::c_int as libc::c_uint {
            ErrorState = SD_ERROR
        } else { ErrorState = SD_BUSY }
    } else { ErrorState = SD_CARD_ERROR as libc::c_int as SD_Error_t }
    return ErrorState;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Gets the SD card status.
  * @retval SD Card error state
  */
#[no_mangle]
pub unsafe extern "C" fn SD_GetCardStatus(mut pCardStatus:
                                              *mut SD_CardStatus_t)
 -> SD_Error_t {
    let mut ErrorState: SD_Error_t = SD_OK;
    let mut Temp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut Status: [uint32_t; 16] = [0; 16];
    let mut Count: uint32_t = 0;
    // Check SD response
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x10000 as
                                              libc::c_uint).wrapping_add(0x2c00
                                                                             as
                                                                             libc::c_uint)
              as *mut SDMMC_TypeDef)).RESP1 &
           0x2000000 as libc::c_int as uint32_t ==
           0x2000000 as libc::c_int as uint32_t {
        return SD_LOCK_UNLOCK_FAILED
    }
    // Set block size for card if it is not equal to current block size for card
    ErrorState =
        SD_TransmitCommand(16 as libc::c_int as uint8_t as libc::c_uint |
                               (0x1 as libc::c_uint) << 6 as libc::c_uint,
                           64 as libc::c_int as uint32_t,
                           1 as libc::c_int as int8_t);
    if ErrorState as libc::c_uint != SD_OK as libc::c_int as libc::c_uint {
        return ErrorState
    }
    // Send CMD55
    ErrorState =
        SD_TransmitCommand(55 as libc::c_int as uint8_t as libc::c_uint |
                               (0x1 as libc::c_uint) << 6 as libc::c_uint,
                           SD_CardRCA, 1 as libc::c_int as int8_t);
    if ErrorState as libc::c_uint != SD_OK as libc::c_int as libc::c_uint {
        return ErrorState
    }
    // Configure the SD DPSM (Data Path State Machine)
    SD_DataTransferInit(64 as libc::c_int as uint32_t,
                        (0x2 as libc::c_uint) << 4 as libc::c_uint |
                            (0x4 as libc::c_uint) << 4 as libc::c_uint,
                        1 as libc::c_int != 0);
    // Send ACMD13 (SD_APP_STAUS)  with argument as card's RCA
    ErrorState =
        SD_TransmitCommand(13 as libc::c_int as uint8_t as libc::c_uint |
                               (0x1 as libc::c_uint) << 6 as libc::c_uint,
                           0 as libc::c_int as uint32_t,
                           1 as libc::c_int as int8_t);
    if ErrorState as libc::c_uint != SD_OK as libc::c_int as libc::c_uint {
        return ErrorState
    }
    // Get status data
    while (*((0x40000000 as
                  libc::c_uint).wrapping_add(0x10000 as
                                                 libc::c_uint).wrapping_add(0x2c00
                                                                                as
                                                                                libc::c_uint)
                 as *mut SDMMC_TypeDef)).STA &
              ((0x1 as libc::c_uint) << 5 as libc::c_uint |
                   (0x1 as libc::c_uint) << 1 as libc::c_uint |
                   (0x1 as libc::c_uint) << 3 as libc::c_uint |
                   (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
              0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x10000 as
                                                  libc::c_uint).wrapping_add(0x2c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut SDMMC_TypeDef)).STA &
               (0x1 as libc::c_uint) << 15 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            Count = 0 as libc::c_int as uint32_t;
            while Count < 8 as libc::c_int as libc::c_uint {
                Status[Count as usize] =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x10000 as
                                                           libc::c_uint).wrapping_add(0x2c00
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut SDMMC_TypeDef)).FIFO;
                Count = Count.wrapping_add(1)
            }
        }
    }
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x10000 as
                                              libc::c_uint).wrapping_add(0x2c00
                                                                             as
                                                                             libc::c_uint)
              as *mut SDMMC_TypeDef)).STA &
           (0x1 as libc::c_uint) << 3 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        return SD_DATA_TIMEOUT
    } else {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x10000 as
                                                  libc::c_uint).wrapping_add(0x2c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut SDMMC_TypeDef)).STA &
               (0x1 as libc::c_uint) << 1 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            return SD_DATA_CRC_FAIL
        } else {
            if (*((0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0x2c00
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut SDMMC_TypeDef)).STA &
                   (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                   0 as libc::c_int as libc::c_uint {
                return SD_RX_OVERRUN
            }
        }
    }
    // Byte 0
    Temp =
        (Status[0 as libc::c_int as usize] &
             0xc0 as libc::c_int as libc::c_uint) >> 6 as libc::c_int;
    (*pCardStatus).DAT_BUS_WIDTH = Temp as uint8_t;
    // Byte 0
    Temp =
        (Status[0 as libc::c_int as usize] &
             0x20 as libc::c_int as libc::c_uint) >> 5 as libc::c_int;
    (*pCardStatus).SECURED_MODE = Temp as uint8_t;
    // Byte 2
    Temp =
        Status[2 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    (*pCardStatus).SD_CARD_TYPE =
        (Temp << 8 as libc::c_int) as uint8_t as uint16_t;
    // Byte 3
    Temp =
        Status[3 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    (*pCardStatus).SD_CARD_TYPE =
        ((*pCardStatus).SD_CARD_TYPE as libc::c_int |
             Temp as uint8_t as libc::c_int) as uint16_t;
    // Byte 4
    Temp =
        Status[4 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    (*pCardStatus).SIZE_OF_PROTECTED_AREA =
        (Temp << 24 as libc::c_int) as uint8_t as uint32_t;
    // Byte 5
    Temp =
        Status[5 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    (*pCardStatus).SIZE_OF_PROTECTED_AREA |=
        (Temp << 16 as libc::c_int) as uint8_t as libc::c_uint;
    // Byte 6
    Temp =
        Status[6 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    (*pCardStatus).SIZE_OF_PROTECTED_AREA |=
        (Temp << 8 as libc::c_int) as uint8_t as libc::c_uint;
    // Byte 7
    Temp =
        Status[7 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    (*pCardStatus).SIZE_OF_PROTECTED_AREA |= Temp as uint8_t as libc::c_uint;
    // Byte 8
    Temp =
        Status[8 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    (*pCardStatus).SPEED_CLASS = Temp as uint8_t;
    // Byte 9
    Temp =
        Status[9 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    (*pCardStatus).PERFORMANCE_MOVE = Temp as uint8_t;
    // Byte 10
    Temp =
        (Status[10 as libc::c_int as usize] &
             0xf0 as libc::c_int as libc::c_uint) >> 4 as libc::c_int;
    (*pCardStatus).AU_SIZE = Temp as uint8_t;
    // Byte 11
    Temp =
        Status[11 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    (*pCardStatus).ERASE_SIZE =
        (Temp << 8 as libc::c_int) as uint8_t as uint16_t;
    // Byte 12
    Temp =
        Status[12 as libc::c_int as usize] &
            0xff as libc::c_int as libc::c_uint;
    (*pCardStatus).ERASE_SIZE =
        ((*pCardStatus).ERASE_SIZE as libc::c_int |
             Temp as uint8_t as libc::c_int) as uint16_t;
    // Byte 13
    Temp =
        (Status[13 as libc::c_int as usize] &
             0xfc as libc::c_int as libc::c_uint) >> 2 as libc::c_int;
    (*pCardStatus).ERASE_TIMEOUT = Temp as uint8_t;
    // Byte 13
    Temp =
        Status[13 as libc::c_int as usize] &
            0x3 as libc::c_int as libc::c_uint;
    (*pCardStatus).ERASE_OFFSET = Temp as uint8_t;
    return SD_OK;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Enquires cards about their operating voltage and configures clock
  *         controls and stores SD information that will be needed in future
  *         in the SD handle.
  * @retval SD Card error state
  */
unsafe extern "C" fn SD_PowerON() -> SD_Error_t {
    let mut ErrorState: SD_Error_t = SD_OK;
    let mut Response: uint32_t = 0;
    let mut Count: uint32_t = 0;
    let mut ValidVoltage: uint32_t = 0;
    let mut SD_Type: uint32_t = 0;
    //uint32_t   TickStart;
    Count = 0 as libc::c_int as uint32_t;
    ValidVoltage = 0 as libc::c_int as uint32_t;
    SD_Type = 0 as libc::c_int as uint32_t;
    // Power ON Sequence -------------------------------------------------------
    let ref mut fresh9 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2c00
                                                                              as
                                                                              libc::c_uint)
               as *mut SDMMC_TypeDef)).CLKCR; // Disable SDMMC1 Clock
    ::core::ptr::write_volatile(fresh9,
                                (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t); // Set Power State to ON
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).POWER as
                                    *mut uint32_t,
                                (0x3 as libc::c_uint) << 0 as libc::c_uint);
    // 1ms: required power up waiting time before starting the SD initialization sequence (make it 2 to be safe)
    delay(2 as libc::c_int as timeMs_t); // Enable SDMMC1 Clock
    let ref mut fresh10 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2c00
                                                                              as
                                                                              libc::c_uint)
               as *mut SDMMC_TypeDef)).CLKCR;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    // CMD0: GO_IDLE_STATE -----------------------------------------------------
    // No CMD response required
    ErrorState =
        SD_TransmitCommand(0 as libc::c_int as uint8_t as uint32_t,
                           0 as libc::c_int as uint32_t,
                           0 as libc::c_int as int8_t);
    if ErrorState as libc::c_uint != SD_OK as libc::c_int as libc::c_uint {
        // CMD Response Timeout (wait for CMDSENT flag)
        return ErrorState
    }
    // CMD8: SEND_IF_COND ------------------------------------------------------
    // Send CMD8 to verify SD card interface operating condition
    // Argument: - [31:12]: Reserved (shall be set to '0')
    //- [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
    //- [7:0]: Check Pattern (recommended 0xAA)
    // CMD Response: R7 */
    ErrorState =
        SD_TransmitCommand(8 as libc::c_int as uint8_t as uint32_t |
                               (0x1 as libc::c_uint) << 6 as libc::c_uint,
                           0x1aa as libc::c_int as uint32_t,
                           7 as libc::c_int as int8_t);
    if ErrorState as libc::c_uint == SD_OK as libc::c_int as libc::c_uint {
        // SD Card 2.0
        SD_CardType = SD_STD_CAPACITY_V2_0;
        SD_Type = 0x40000000 as libc::c_int as uint32_t
    }
    // Send CMD55
    // If ErrorState is Command Timeout, it is a MMC card
    // If ErrorState is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch) or SD card 1.x
    ErrorState =
        SD_TransmitCommand(55 as libc::c_int as uint8_t as libc::c_uint |
                               (0x1 as libc::c_uint) << 6 as libc::c_uint,
                           0 as libc::c_int as uint32_t,
                           1 as libc::c_int as int8_t); // else MMC Card
    if ErrorState as libc::c_uint == SD_OK as libc::c_int as libc::c_uint {
        // SD CARD
        // Send ACMD41 SD_APP_OP_COND with Argument 0x80100000
        while ValidVoltage == 0 as libc::c_int as libc::c_uint &&
                  Count < 0xffff as libc::c_int as uint32_t {
            // SEND CMD55 APP_CMD with RCA as 0
            ErrorState =
                SD_TransmitCommand(55 as libc::c_int as uint8_t as
                                       libc::c_uint |
                                       (0x1 as libc::c_uint) <<
                                           6 as libc::c_uint,
                                   0 as libc::c_int as uint32_t,
                                   1 as libc::c_int as int8_t);
            if ErrorState as libc::c_uint !=
                   SD_OK as libc::c_int as libc::c_uint {
                return ErrorState
            }
            // Send CMD41
            ErrorState =
                SD_TransmitCommand(41 as libc::c_int as uint8_t as
                                       libc::c_uint |
                                       (0x1 as libc::c_uint) <<
                                           6 as libc::c_uint,
                                   0x80100000 as libc::c_uint | SD_Type,
                                   3 as libc::c_int as
                                       int8_t); // Get command response
            if ErrorState as libc::c_uint !=
                   SD_OK as libc::c_int as libc::c_uint {
                return ErrorState
            } // Get operating voltage
            Response =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x10000 as
                                                       libc::c_uint).wrapping_add(0x2c00
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut SDMMC_TypeDef)).RESP1;
            ValidVoltage =
                if Response >> 31 as libc::c_int ==
                       1 as libc::c_int as libc::c_uint {
                    1 as libc::c_int
                } else { 0 as libc::c_int } as uint32_t;
            Count = Count.wrapping_add(1)
        }
        if Count >= 0xffff as libc::c_int as uint32_t {
            return SD_INVALID_VOLTRANGE
        }
        if Response & 0x40000000 as libc::c_int as uint32_t ==
               0x40000000 as libc::c_int as uint32_t {
            SD_CardType = SD_HIGH_CAPACITY
        }
    }
    return ErrorState;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Turns the SDMMC1 output signals off.
  * @retval SD Card error state
  */
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Finds the SD card SCR register value.
  * @param  pSCR: pointer to the buffer that will contain the SCR value
  * @retval SD Card error state
  */
unsafe extern "C" fn SD_FindSCR(mut pSCR: *mut uint32_t) -> SD_Error_t {
    let mut ErrorState: SD_Error_t = SD_OK;
    let mut Index: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tempscr: [uint32_t; 2] =
        [0 as libc::c_int as uint32_t, 0 as libc::c_int as uint32_t];
    // Set Block Size To 8 Bytes
    // Send CMD55 APP_CMD with argument as card's RCA
    ErrorState =
        SD_TransmitCommand(16 as libc::c_int as uint8_t as libc::c_uint |
                               (0x1 as libc::c_uint) << 6 as libc::c_uint,
                           8 as libc::c_int as uint32_t,
                           1 as libc::c_int as int8_t);
    if ErrorState as libc::c_uint == SD_OK as libc::c_int as libc::c_uint {
        // Send CMD55 APP_CMD with argument as card's RCA
        ErrorState =
            SD_TransmitCommand(55 as libc::c_int as uint8_t as libc::c_uint |
                                   (0x1 as libc::c_uint) << 6 as libc::c_uint,
                               SD_CardRCA, 1 as libc::c_int as int8_t);
        if ErrorState as libc::c_uint == SD_OK as libc::c_int as libc::c_uint
           {
            SD_DataTransferInit(8 as libc::c_int as uint32_t,
                                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                                    (0x2 as libc::c_uint) <<
                                        4 as libc::c_uint,
                                1 as libc::c_int != 0);
            // Send ACMD51 SD_APP_SEND_SCR with argument as 0
            ErrorState =
                SD_TransmitCommand(51 as libc::c_int as uint8_t as
                                       libc::c_uint |
                                       (0x1 as libc::c_uint) <<
                                           6 as libc::c_uint,
                                   0 as libc::c_int as uint32_t,
                                   1 as libc::c_int as int8_t);
            if ErrorState as libc::c_uint ==
                   SD_OK as libc::c_int as libc::c_uint {
                while (*((0x40000000 as
                              libc::c_uint).wrapping_add(0x10000 as
                                                             libc::c_uint).wrapping_add(0x2c00
                                                                                            as
                                                                                            libc::c_uint)
                             as *mut SDMMC_TypeDef)).STA &
                          ((0x1 as libc::c_uint) << 5 as libc::c_uint |
                               (0x1 as libc::c_uint) << 1 as libc::c_uint |
                               (0x1 as libc::c_uint) << 3 as libc::c_uint |
                               (0x1 as libc::c_uint) << 10 as libc::c_uint) ==
                          0 as libc::c_int as libc::c_uint {
                    if (*((0x40000000 as
                               libc::c_uint).wrapping_add(0x10000 as
                                                              libc::c_uint).wrapping_add(0x2c00
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut SDMMC_TypeDef)).STA &
                           (0x1 as libc::c_uint) << 21 as libc::c_uint !=
                           0 as libc::c_int as libc::c_uint {
                        *tempscr.as_mut_ptr().offset(Index as isize) =
                            (*((0x40000000 as
                                    libc::c_uint).wrapping_add(0x10000 as
                                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                                  as
                                                                                                  libc::c_uint)
                                   as *mut SDMMC_TypeDef)).FIFO;
                        Index = Index.wrapping_add(1)
                    }
                }
                if (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x10000 as
                                                          libc::c_uint).wrapping_add(0x2c00
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut SDMMC_TypeDef)).STA &
                       (0x1 as libc::c_uint) << 3 as libc::c_uint !=
                       0 as libc::c_int as libc::c_uint {
                    ErrorState = SD_DATA_TIMEOUT
                } else if (*((0x40000000 as
                                  libc::c_uint).wrapping_add(0x10000 as
                                                                 libc::c_uint).wrapping_add(0x2c00
                                                                                                as
                                                                                                libc::c_uint)
                                 as *mut SDMMC_TypeDef)).STA &
                              (0x1 as libc::c_uint) << 1 as libc::c_uint !=
                              0 as libc::c_int as libc::c_uint {
                    ErrorState = SD_DATA_CRC_FAIL
                } else if (*((0x40000000 as
                                  libc::c_uint).wrapping_add(0x10000 as
                                                                 libc::c_uint).wrapping_add(0x2c00
                                                                                                as
                                                                                                libc::c_uint)
                                 as *mut SDMMC_TypeDef)).STA &
                              (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                              0 as libc::c_int as libc::c_uint {
                    ErrorState = SD_RX_OVERRUN
                } else if (*((0x40000000 as
                                  libc::c_uint).wrapping_add(0x10000 as
                                                                 libc::c_uint).wrapping_add(0x2c00
                                                                                                as
                                                                                                libc::c_uint)
                                 as *mut SDMMC_TypeDef)).STA &
                              (0x1 as libc::c_uint) << 21 as libc::c_uint !=
                              0 as libc::c_int as libc::c_uint {
                    ErrorState = SD_OUT_OF_BOUND
                } else {
                    *pSCR.offset(1 as libc::c_int as isize) =
                        (tempscr[0 as libc::c_int as usize] &
                             0xff as libc::c_int as uint32_t) <<
                            24 as libc::c_int |
                            (tempscr[0 as libc::c_int as usize] &
                                 0xff00 as libc::c_int as uint32_t) <<
                                8 as libc::c_int |
                            (tempscr[0 as libc::c_int as usize] &
                                 0xff0000 as libc::c_int as uint32_t) >>
                                8 as libc::c_int |
                            (tempscr[0 as libc::c_int as usize] &
                                 0xff000000 as libc::c_uint) >>
                                24 as libc::c_int;
                    *pSCR =
                        (tempscr[1 as libc::c_int as usize] &
                             0xff as libc::c_int as uint32_t) <<
                            24 as libc::c_int |
                            (tempscr[1 as libc::c_int as usize] &
                                 0xff00 as libc::c_int as uint32_t) <<
                                8 as libc::c_int |
                            (tempscr[1 as libc::c_int as usize] &
                                 0xff0000 as libc::c_int as uint32_t) >>
                                8 as libc::c_int |
                            (tempscr[1 as libc::c_int as usize] &
                                 0xff000000 as libc::c_uint) >>
                                24 as libc::c_int
                }
            }
        }
    }
    return ErrorState;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Checks if the SD card is in programming state.
  * @param  pStatus: pointer to the variable that will contain the SD card state
  * @retval SD Card error state
  */
/*
static SD_Error_t SD_IsCardProgramming(uint8_t *pStatus)
{
    uint32_t Response_R1;

    SD_TransmitCommand((SD_CMD_SEND_STATUS | SDMMC_CMD_RESPONSE_SHORT), SD_CardRCA, 0);
    if((SDMMC1->STA & SDMMC_STA_CTIMEOUT) != 0)         return SD_CMD_RSP_TIMEOUT;
    else if((SDMMC1->STA & SDMMC_STA_CCRCFAIL) != 0)    return SD_CMD_CRC_FAIL;
    if((uint32_t)SDMMC1->RESPCMD != SD_CMD_SEND_STATUS) return SD_ILLEGAL_CMD;  // Check if is of desired command
    Response_R1 = SDMMC1->RESP1;                                                // We have received response, retrieve it for analysis
    *pStatus = (uint8_t)((Response_R1 >> 9) & 0x0000000F);                      // Find out card status

    return CheckOCR_Response(Response_R1);
}
*/
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  Initialize the SDMMC1 module, DMA, and IO
  */
#[no_mangle]
pub unsafe extern "C" fn SD_Initialize_LL(mut dma: *mut DMA_Stream_TypeDef) {
    // Reset SDMMC1 Module
    let ref mut fresh11 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2RSTR;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         11 as libc::c_uint) as uint32_t as
                                    uint32_t);
    delay(1 as libc::c_int as timeMs_t);
    let ref mut fresh12 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2RSTR;
    ::core::ptr::write_volatile(fresh12,
                                (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           11 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    delay(1 as libc::c_int as timeMs_t);
    // Enable SDMMC1 clock
    let ref mut fresh13 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2ENR;
    ::core::ptr::write_volatile(fresh13,
                                (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         11 as libc::c_uint) as uint32_t as
                                    uint32_t);
    // Enable DMA2 clocks
    let ref mut fresh14 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh14,
                                (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         22 as libc::c_uint) as uint32_t as
                                    uint32_t);
    //Configure Pins
    let ref mut fresh15 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh15,
                                (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          2 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              3 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    let d0: IO_t =
        IOGetByTag(((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 8 as libc::c_int) as ioTag_t);
    let d1: IO_t =
        IOGetByTag(((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 9 as libc::c_int) as ioTag_t);
    let d2: IO_t =
        IOGetByTag(((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 10 as libc::c_int) as ioTag_t);
    let d3: IO_t =
        IOGetByTag(((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 11 as libc::c_int) as ioTag_t);
    let clk: IO_t =
        IOGetByTag(((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 12 as libc::c_int) as ioTag_t);
    let cmd: IO_t =
        IOGetByTag(((3 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 2 as libc::c_int) as ioTag_t);
    IOInit(d0, OWNER_SDCARD, 0 as libc::c_int as uint8_t);
    IOInit(d1, OWNER_SDCARD, 0 as libc::c_int as uint8_t);
    IOInit(d2, OWNER_SDCARD, 0 as libc::c_int as uint8_t);
    IOInit(d3, OWNER_SDCARD, 0 as libc::c_int as uint8_t);
    IOInit(clk, OWNER_SDCARD, 0 as libc::c_int as uint8_t);
    IOInit(cmd, OWNER_SDCARD, 0 as libc::c_int as uint8_t);
    IOConfigGPIOAF(d0,
                   (0x2 as libc::c_uint |
                        (0x3 as libc::c_uint) << 2 as libc::c_int |
                        (0 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, 0xc as libc::c_uint as uint8_t);
    IOConfigGPIOAF(d1,
                   (0x2 as libc::c_uint |
                        (0x3 as libc::c_uint) << 2 as libc::c_int |
                        (0 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, 0xc as libc::c_uint as uint8_t);
    IOConfigGPIOAF(d2,
                   (0x2 as libc::c_uint |
                        (0x3 as libc::c_uint) << 2 as libc::c_int |
                        (0 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, 0xc as libc::c_uint as uint8_t);
    IOConfigGPIOAF(d3,
                   (0x2 as libc::c_uint |
                        (0x3 as libc::c_uint) << 2 as libc::c_int |
                        (0 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, 0xc as libc::c_uint as uint8_t);
    IOConfigGPIOAF(clk,
                   (0x2 as libc::c_uint |
                        (0x3 as libc::c_uint) << 2 as libc::c_int |
                        (0 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, 0xc as libc::c_uint as uint8_t);
    IOConfigGPIOAF(cmd,
                   (0x2 as libc::c_uint |
                        (0x3 as libc::c_uint) << 2 as libc::c_int |
                        (0 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, 0xc as libc::c_uint as uint8_t);
    let mut PriorityGroup: uint32_t = __NVIC_GetPriorityGrouping();
    // NVIC configuration for SDIO interrupts
    __NVIC_SetPriority(SDMMC1_IRQn,
                       NVIC_EncodePriority(PriorityGroup,
                                           1 as libc::c_int as uint32_t,
                                           0 as libc::c_int as uint32_t));
    __NVIC_EnableIRQ(SDMMC1_IRQn);
    dma_stream = dma;
    let ref mut fresh16 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1ENR;
    ::core::ptr::write_volatile(fresh16,
                                (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         22 as libc::c_uint) as uint32_t as
                                    uint32_t);
    if dma_stream as uint32_t ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x6400
                                                                              as
                                                                              libc::c_uint).wrapping_add(0x58
                                                                                                             as
                                                                                                             libc::c_uint)
               as *mut DMA_Stream_TypeDef as uint32_t {
        // Initialize DMA2 channel 3
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x58
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as
                                                *mut DMA_Stream_TypeDef)).CR
                                        as *mut uint32_t,
                                    0 as libc::c_int as
                                        uint32_t); // Reset DMA Stream control register
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x58
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as
                                                *mut DMA_Stream_TypeDef)).PAR
                                        as *mut uint32_t,
                                    &mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x10000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x2c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut SDMMC_TypeDef)).FIFO
                                        as *mut uint32_t as
                                        uint32_t); // Clear all interrupt flags
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef)).LIFCR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        27 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            26 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            25 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            24 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            22 as
                                                libc::c_uint); // Configuration FIFO control register
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x58
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as
                                                *mut DMA_Stream_TypeDef)).CR
                                        as *mut uint32_t,
                                    0x8000000 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            10 as libc::c_uint |
                                        (0x2 as libc::c_uint) <<
                                            11 as libc::c_uint |
                                        (0x2 as libc::c_uint) <<
                                            13 as libc::c_uint |
                                        (0x3 as libc::c_uint) <<
                                            16 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            23 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            21 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            6 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x58
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as
                                                *mut DMA_Stream_TypeDef)).FCR
                                        as *mut uint32_t,
                                    (0x1 as libc::c_uint) << 2 as libc::c_uint
                                        |
                                        (0x3 as libc::c_uint) <<
                                            0 as libc::c_uint);
        dmaInit(dmaGetIdentifier((0x40000000 as
                                      libc::c_uint).wrapping_add(0x20000 as
                                                                     libc::c_uint).wrapping_add(0x6400
                                                                                                    as
                                                                                                    libc::c_uint).wrapping_add(0x58
                                                                                                                                   as
                                                                                                                                   libc::c_uint)
                                     as *mut DMA_Stream_TypeDef),
                OWNER_SDCARD, 0 as libc::c_int as uint8_t);
        dmaSetHandler(dmaGetIdentifier((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x6400
                                                                                                          as
                                                                                                          libc::c_uint).wrapping_add(0x58
                                                                                                                                         as
                                                                                                                                         libc::c_uint)
                                           as *mut DMA_Stream_TypeDef),
                      Some(SDMMC_DMA_ST3_IRQHandler as
                               unsafe extern "C" fn(_:
                                                        *mut dmaChannelDescriptor_t)
                                   -> ()), 1 as libc::c_int as uint32_t,
                      0 as libc::c_int as uint32_t);
    } else {
        // Initialize DMA2 channel 6
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0xa0
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as
                                                *mut DMA_Stream_TypeDef)).CR
                                        as *mut uint32_t,
                                    0 as libc::c_int as
                                        uint32_t); // Reset DMA Stream control register
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0xa0
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as
                                                *mut DMA_Stream_TypeDef)).PAR
                                        as *mut uint32_t,
                                    &mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x10000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x2c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut SDMMC_TypeDef)).FIFO
                                        as *mut uint32_t as
                                        uint32_t); // Clear all interrupt flags
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef)).HIFCR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        21 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            20 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            19 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            18 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            16 as
                                                libc::c_uint); // Configuration FIFO control register
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0xa0
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as
                                                *mut DMA_Stream_TypeDef)).CR
                                        as *mut uint32_t,
                                    0x8000000 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            10 as libc::c_uint |
                                        (0x2 as libc::c_uint) <<
                                            11 as libc::c_uint |
                                        (0x2 as libc::c_uint) <<
                                            13 as libc::c_uint |
                                        (0x3 as libc::c_uint) <<
                                            16 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            23 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            21 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            6 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0xa0
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as
                                                *mut DMA_Stream_TypeDef)).FCR
                                        as *mut uint32_t,
                                    (0x1 as libc::c_uint) << 2 as libc::c_uint
                                        |
                                        (0x3 as libc::c_uint) <<
                                            0 as libc::c_uint);
        dmaInit(dmaGetIdentifier((0x40000000 as
                                      libc::c_uint).wrapping_add(0x20000 as
                                                                     libc::c_uint).wrapping_add(0x6400
                                                                                                    as
                                                                                                    libc::c_uint).wrapping_add(0xa0
                                                                                                                                   as
                                                                                                                                   libc::c_uint)
                                     as *mut DMA_Stream_TypeDef),
                OWNER_SDCARD, 0 as libc::c_int as uint8_t);
        dmaSetHandler(dmaGetIdentifier((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x6400
                                                                                                          as
                                                                                                          libc::c_uint).wrapping_add(0xa0
                                                                                                                                         as
                                                                                                                                         libc::c_uint)
                                           as *mut DMA_Stream_TypeDef),
                      Some(SDMMC_DMA_ST6_IRQHandler as
                               unsafe extern "C" fn(_:
                                                        *mut dmaChannelDescriptor_t)
                                   -> ()), 1 as libc::c_int as uint32_t,
                      0 as libc::c_int as uint32_t);
    };
}
/* * -----------------------------------------------------------------------------------------------------------------*/
#[no_mangle]
pub unsafe extern "C" fn SD_GetState() -> bool {
    // Check SDCARD status
    if SD_GetStatus() as libc::c_uint == SD_OK as libc::c_int as libc::c_uint
       {
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
/* Prototype(s) -----------------------------------------------------------------------------------------------------*/
/* * -----------------------------------------------------------------------------------------------------------------*/
#[no_mangle]
pub unsafe extern "C" fn SD_Init() -> bool {
    let mut ErrorState: SD_Error_t = SD_OK;
    // Check if SD card is present
    if SD_IsDetected() as libc::c_int !=
           0x1 as libc::c_int as uint8_t as libc::c_int {
        return 0 as libc::c_int != 0
    }
    // Initialize SDMMC1 peripheral interface with default configuration for SD card initialization
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).CLKCR as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x10000 as
                                                                       libc::c_uint).wrapping_add(0x2c00
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut SDMMC_TypeDef)).CLKCR &
                                    !((0xff as libc::c_uint) <<
                                          0 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              9 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              10 as libc::c_uint |
                                          (0x3 as libc::c_uint) <<
                                              11 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              13 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              14 as libc::c_uint) |
                                    0x76 as libc::c_int as uint8_t as
                                        uint32_t);
    ErrorState = SD_PowerON();
    if ErrorState as libc::c_uint == SD_OK as libc::c_int as libc::c_uint {
        // Identify card operating voltage
        ErrorState = SD_InitializeCard();
        if ErrorState as libc::c_uint == SD_OK as libc::c_int as libc::c_uint
           {
            // Initialize the present card and put them in idle state
            ErrorState = SD_GetCardInfo();
            if ErrorState as libc::c_uint ==
                   SD_OK as libc::c_int as libc::c_uint {
                // Read CSD/CID MSD registers
                // Select the Card - Send CMD7 SDMMC_SEL_DESEL_CARD
                ErrorState =
                    SD_TransmitCommand(7 as libc::c_int as uint8_t as
                                           libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               6 as libc::c_uint, SD_CardRCA,
                                       1 as libc::c_int as int8_t);
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x10000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x2c00
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut SDMMC_TypeDef)).CLKCR
                                                as *mut uint32_t,
                                            (*((0x40000000 as
                                                    libc::c_uint).wrapping_add(0x10000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                                                  as
                                                                                                                  libc::c_uint)
                                                   as
                                                   *mut SDMMC_TypeDef)).CLKCR
                                                &
                                                !((0xff as libc::c_uint) <<
                                                      0 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          9 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          10 as libc::c_uint |
                                                      (0x3 as libc::c_uint) <<
                                                          11 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          13 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          14 as libc::c_uint)
                                                |
                                                0 as libc::c_int as uint8_t as
                                                    uint32_t)
            }
        }
    }
    // Configure SD Bus width
    if ErrorState as libc::c_uint == SD_OK as libc::c_int as libc::c_uint {
        // Enable wide operation
        ErrorState =
            SD_WideBusOperationConfig((0x1 as libc::c_uint) <<
                                          11 as libc::c_uint)
    }
    // Configure the SDCARD device
    return ErrorState as u64 != 0;
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  This function handles SD card interrupt request.
  */
#[no_mangle]
pub unsafe extern "C" fn SDMMC1_IRQHandler() {
    // Check for SDMMC1 interrupt flags
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x10000 as
                                              libc::c_uint).wrapping_add(0x2c00
                                                                             as
                                                                             libc::c_uint)
              as *mut SDMMC_TypeDef)).STA &
           (0x1 as libc::c_uint) << 8 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x10000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x2c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut SDMMC_TypeDef)).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        8 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x10000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x2c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut SDMMC_TypeDef)).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) << 0 as libc::c_uint
                                        |
                                        (0x1 as libc::c_uint) <<
                                            1 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            2 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            3 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            4 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            6 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            7 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            8 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            10 as libc::c_uint);
        let ref mut fresh17 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut SDMMC_TypeDef)).MASK;
        ::core::ptr::write_volatile(fresh17,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               8 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   1 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   3 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   4 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   5 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   14 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        // No transfer error
        if SD_Handle.Operation & 0x2 as libc::c_int as libc::c_uint ==
               ((1 as libc::c_int) << 1 as libc::c_int) as libc::c_uint {
            /* Currently doesn't implement multiple block write handling */
            /* Disable the stream */
            ::core::ptr::write_volatile(&mut (*dma_stream).CR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*dma_stream).CR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            let ref mut fresh18 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x10000 as
                                                       libc::c_uint).wrapping_add(0x2c00
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut SDMMC_TypeDef)).DCTRL;
            ::core::ptr::write_volatile(fresh18,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   3 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Transfer is complete */
            ::core::ptr::write_volatile(&mut SD_Handle.TXCplt as
                                            *mut uint32_t,
                                        0 as libc::c_int as uint32_t);
            if SD_Handle.Operation & 0x1 as libc::c_int as libc::c_uint ==
                   SD_MULTIPLE_BLOCK as libc::c_int as libc::c_uint {
                /* Send stop command in multiblock write */
                SD_TransmitCommand(12 as libc::c_int as uint8_t as
                                       libc::c_uint |
                                       (0x1 as libc::c_uint) <<
                                           6 as libc::c_uint,
                                   0 as libc::c_int as uint32_t,
                                   1 as libc::c_int as int8_t);
            }
        }
        ::core::ptr::write_volatile(&mut SD_Handle.TransferComplete as
                                        *mut uint32_t,
                                    1 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut SD_Handle.TransferError as
                                        *mut uint32_t,
                                    SD_OK as libc::c_int as uint32_t)
    } else if (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x10000 as
                                                     libc::c_uint).wrapping_add(0x2c00
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut SDMMC_TypeDef)).STA &
                  (0x1 as libc::c_uint) << 1 as libc::c_uint !=
                  0 as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut SD_Handle.TransferError as
                                        *mut uint32_t,
                                    SD_DATA_CRC_FAIL as libc::c_int as
                                        uint32_t)
    } else if (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x10000 as
                                                     libc::c_uint).wrapping_add(0x2c00
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut SDMMC_TypeDef)).STA &
                  (0x1 as libc::c_uint) << 3 as libc::c_uint !=
                  0 as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut SD_Handle.TransferError as
                                        *mut uint32_t,
                                    SD_DATA_TIMEOUT as libc::c_int as
                                        uint32_t)
    } else if (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x10000 as
                                                     libc::c_uint).wrapping_add(0x2c00
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut SDMMC_TypeDef)).STA &
                  (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                  0 as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut SD_Handle.TransferError as
                                        *mut uint32_t,
                                    SD_RX_OVERRUN as libc::c_int as uint32_t)
    } else if (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x10000 as
                                                     libc::c_uint).wrapping_add(0x2c00
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut SDMMC_TypeDef)).STA &
                  (0x1 as libc::c_uint) << 4 as libc::c_uint !=
                  0 as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut SD_Handle.TransferError as
                                        *mut uint32_t,
                                    SD_TX_UNDERRUN as libc::c_int as uint32_t)
    }
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut SDMMC_TypeDef)).ICR as
                                    *mut uint32_t,
                                (0x1 as libc::c_uint) << 0 as libc::c_uint |
                                    (0x1 as libc::c_uint) << 1 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 2 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 3 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 4 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 5 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 6 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 7 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 8 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) <<
                                        10 as libc::c_uint);
    // Disable all SDMMC1 peripheral interrupt sources
    let ref mut fresh19 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2c00
                                                                              as
                                                                              libc::c_uint)
               as *mut SDMMC_TypeDef)).MASK;
    ::core::ptr::write_volatile(fresh19,
                                (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               3 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               8 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               14 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               15 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               4 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                    as uint32_t);
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  This function handles DMA2 Stream 3 interrupt request.
  */
#[no_mangle]
pub unsafe extern "C" fn SDMMC_DMA_ST3_IRQHandler(mut dma:
                                                      *mut dmaChannelDescriptor_t) {
    // Transfer Error Interrupt management
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x6400
                                                                             as
                                                                             libc::c_uint)
              as *mut DMA_TypeDef)).LISR &
           (0x1 as libc::c_uint) << 25 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x6400
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(0x58
                                                                                                                as
                                                                                                                libc::c_uint)
                  as *mut DMA_Stream_TypeDef)).CR &
               (0x1 as libc::c_uint) << 2 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh20 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6400
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0x58
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as
                       *mut DMA_Stream_TypeDef)).CR; // Disable the transfer error interrupt
            ::core::ptr::write_volatile(fresh20,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).LIFCR
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            25 as libc::c_uint)
            // Clear the transfer error flag
        }
    }
    // FIFO Error Interrupt management
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x6400
                                                                             as
                                                                             libc::c_uint)
              as *mut DMA_TypeDef)).LISR &
           (0x1 as libc::c_uint) << 22 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x6400
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(0x58
                                                                                                                as
                                                                                                                libc::c_uint)
                  as *mut DMA_Stream_TypeDef)).FCR &
               (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh21 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6400
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0x58
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as
                       *mut DMA_Stream_TypeDef)).FCR; // Disable the FIFO Error interrupt
            ::core::ptr::write_volatile(fresh21,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   7 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).LIFCR
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            22 as libc::c_uint)
            // Clear the FIFO error flag
        }
    }
    // Direct Mode Error Interrupt management
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x6400
                                                                             as
                                                                             libc::c_uint)
              as *mut DMA_TypeDef)).LISR &
           (0x1 as libc::c_uint) << 24 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x6400
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(0x58
                                                                                                                as
                                                                                                                libc::c_uint)
                  as *mut DMA_Stream_TypeDef)).CR &
               (0x1 as libc::c_uint) << 1 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh22 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6400
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0x58
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as
                       *mut DMA_Stream_TypeDef)).CR; // Disable the direct mode Error interrupt
            ::core::ptr::write_volatile(fresh22,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   1 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).LIFCR
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            24 as libc::c_uint)
            // Clear the FIFO error flag
        }
    }
    // Half Transfer Complete Interrupt management
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x6400
                                                                             as
                                                                             libc::c_uint)
              as *mut DMA_TypeDef)).LISR &
           (0x1 as libc::c_uint) << 26 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x6400
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(0x58
                                                                                                                as
                                                                                                                libc::c_uint)
                  as *mut DMA_Stream_TypeDef)).CR &
               (0x1 as libc::c_uint) << 3 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            if (*((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x6400
                                                                                     as
                                                                                     libc::c_uint).wrapping_add(0x58
                                                                                                                    as
                                                                                                                    libc::c_uint)
                      as *mut DMA_Stream_TypeDef)).CR &
                   (0x1 as libc::c_uint) << 18 as libc::c_uint !=
                   0 as libc::c_int as libc::c_uint {
                // Multi_Buffering mode enabled
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).LIFCR
                                                as *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                26 as libc::c_uint)
                // Clear the half transfer complete flag
            } else {
                if (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x6400
                                                                                         as
                                                                                         libc::c_uint).wrapping_add(0x58
                                                                                                                        as
                                                                                                                        libc::c_uint)
                          as *mut DMA_Stream_TypeDef)).CR &
                       (0x1 as libc::c_uint) << 8 as libc::c_uint ==
                       0 as libc::c_int as libc::c_uint {
                    // Disable the half transfer interrupt if the DMA mode is not CIRCULAR
                    let ref mut fresh23 =
                        (*((0x40000000 as
                                libc::c_uint).wrapping_add(0x20000 as
                                                               libc::c_uint).wrapping_add(0x6400
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x58
                                                                                                                             as
                                                                                                                             libc::c_uint)
                               as *mut DMA_Stream_TypeDef)).CR;
                    ::core::ptr::write_volatile(fresh23,
                                                (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           3 as libc::c_uint))
                                                    as uint32_t as uint32_t)
                    // Disable the half transfer interrupt
                }
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).LIFCR
                                                as *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                26 as libc::c_uint)
                // Clear the half transfer complete flag
            }
        }
    }
    // Transfer Complete Interrupt management
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x6400
                                                                             as
                                                                             libc::c_uint)
              as *mut DMA_TypeDef)).LISR &
           (0x1 as libc::c_uint) << 27 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x6400
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(0x58
                                                                                                                as
                                                                                                                libc::c_uint)
                  as *mut DMA_Stream_TypeDef)).CR &
               (0x1 as libc::c_uint) << 4 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            if (*((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x6400
                                                                                     as
                                                                                     libc::c_uint).wrapping_add(0x58
                                                                                                                    as
                                                                                                                    libc::c_uint)
                      as *mut DMA_Stream_TypeDef)).CR &
                   (0x1 as libc::c_uint) << 18 as libc::c_uint !=
                   0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).LIFCR
                                                as *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                27 as libc::c_uint)
                // Clear the transfer complete flag
            } else {
                //Disable the transfer complete interrupt if the DMA mode is not CIRCULAR
                if (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x6400
                                                                                         as
                                                                                         libc::c_uint).wrapping_add(0x58
                                                                                                                        as
                                                                                                                        libc::c_uint)
                          as *mut DMA_Stream_TypeDef)).CR &
                       (0x1 as libc::c_uint) << 8 as libc::c_uint ==
                       0 as libc::c_int as libc::c_uint {
                    let ref mut fresh24 =
                        (*((0x40000000 as
                                libc::c_uint).wrapping_add(0x20000 as
                                                               libc::c_uint).wrapping_add(0x6400
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x58
                                                                                                                             as
                                                                                                                             libc::c_uint)
                               as *mut DMA_Stream_TypeDef)).CR;
                    ::core::ptr::write_volatile(fresh24,
                                                (::core::ptr::read_volatile::<uint32_t>(fresh24
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           4 as libc::c_uint))
                                                    as uint32_t as uint32_t)
                    // Disable the transfer complete interrupt
                } // Clear the transfer complete flag
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).LIFCR
                                                as *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                27 as libc::c_uint);
                SD_DMA_Complete((0x40000000 as
                                     libc::c_uint).wrapping_add(0x20000 as
                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                   as
                                                                                                   libc::c_uint).wrapping_add(0x58
                                                                                                                                  as
                                                                                                                                  libc::c_uint)
                                    as *mut DMA_Stream_TypeDef);
            }
        }
    };
}
/* * -----------------------------------------------------------------------------------------------------------------*/
/* *
  * @brief  This function handles DMA2 Stream 6 interrupt request.
  */
#[no_mangle]
pub unsafe extern "C" fn SDMMC_DMA_ST6_IRQHandler(mut dma:
                                                      *mut dmaChannelDescriptor_t) {
    // Transfer Error Interrupt management
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x6400
                                                                             as
                                                                             libc::c_uint)
              as *mut DMA_TypeDef)).HISR &
           (0x1 as libc::c_uint) << 19 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x6400
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                as
                                                                                                                libc::c_uint)
                  as *mut DMA_Stream_TypeDef)).CR &
               (0x1 as libc::c_uint) << 2 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh25 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6400
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0xa0
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as
                       *mut DMA_Stream_TypeDef)).CR; // Disable the transfer error interrupt
            ::core::ptr::write_volatile(fresh25,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh25
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).HIFCR
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            19 as libc::c_uint)
            // Clear the transfer error flag
        }
    }
    // FIFO Error Interrupt management
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x6400
                                                                             as
                                                                             libc::c_uint)
              as *mut DMA_TypeDef)).HISR &
           (0x1 as libc::c_uint) << 16 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x6400
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                as
                                                                                                                libc::c_uint)
                  as *mut DMA_Stream_TypeDef)).FCR &
               (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh26 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6400
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0xa0
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as
                       *mut DMA_Stream_TypeDef)).FCR; // Disable the FIFO Error interrupt
            ::core::ptr::write_volatile(fresh26,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh26
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   7 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).HIFCR
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            16 as libc::c_uint)
            // Clear the FIFO error flag
        }
    }
    // Direct Mode Error Interrupt management
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x6400
                                                                             as
                                                                             libc::c_uint)
              as *mut DMA_TypeDef)).HISR &
           (0x1 as libc::c_uint) << 18 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x6400
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                as
                                                                                                                libc::c_uint)
                  as *mut DMA_Stream_TypeDef)).CR &
               (0x1 as libc::c_uint) << 1 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh27 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6400
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0xa0
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as
                       *mut DMA_Stream_TypeDef)).CR; // Disable the direct mode Error interrupt
            ::core::ptr::write_volatile(fresh27,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh27
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   1 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).HIFCR
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            18 as libc::c_uint)
            // Clear the FIFO error flag
        }
    }
    // Half Transfer Complete Interrupt management
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x6400
                                                                             as
                                                                             libc::c_uint)
              as *mut DMA_TypeDef)).HISR &
           (0x1 as libc::c_uint) << 20 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x6400
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                as
                                                                                                                libc::c_uint)
                  as *mut DMA_Stream_TypeDef)).CR &
               (0x1 as libc::c_uint) << 3 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            if (*((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x6400
                                                                                     as
                                                                                     libc::c_uint).wrapping_add(0xa0
                                                                                                                    as
                                                                                                                    libc::c_uint)
                      as *mut DMA_Stream_TypeDef)).CR &
                   (0x1 as libc::c_uint) << 18 as libc::c_uint !=
                   0 as libc::c_int as libc::c_uint {
                // Multi_Buffering mode enabled
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).HIFCR
                                                as *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                20 as libc::c_uint)
                // Clear the half transfer complete flag
            } else {
                if (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x6400
                                                                                         as
                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                        as
                                                                                                                        libc::c_uint)
                          as *mut DMA_Stream_TypeDef)).CR &
                       (0x1 as libc::c_uint) << 8 as libc::c_uint ==
                       0 as libc::c_int as libc::c_uint {
                    // Disable the half transfer interrupt if the DMA mode is not CIRCULAR
                    let ref mut fresh28 =
                        (*((0x40000000 as
                                libc::c_uint).wrapping_add(0x20000 as
                                                               libc::c_uint).wrapping_add(0x6400
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0xa0
                                                                                                                             as
                                                                                                                             libc::c_uint)
                               as *mut DMA_Stream_TypeDef)).CR;
                    ::core::ptr::write_volatile(fresh28,
                                                (::core::ptr::read_volatile::<uint32_t>(fresh28
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           3 as libc::c_uint))
                                                    as uint32_t as uint32_t)
                    // Disable the half transfer interrupt
                }
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).HIFCR
                                                as *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                20 as libc::c_uint)
                // Clear the half transfer complete flag
            }
        }
    }
    // Transfer Complete Interrupt management
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x6400
                                                                             as
                                                                             libc::c_uint)
              as *mut DMA_TypeDef)).HISR &
           (0x1 as libc::c_uint) << 21 as libc::c_uint !=
           0 as libc::c_int as libc::c_uint {
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x6400
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                as
                                                                                                                libc::c_uint)
                  as *mut DMA_Stream_TypeDef)).CR &
               (0x1 as libc::c_uint) << 4 as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            if (*((0x40000000 as
                       libc::c_uint).wrapping_add(0x20000 as
                                                      libc::c_uint).wrapping_add(0x6400
                                                                                     as
                                                                                     libc::c_uint).wrapping_add(0xa0
                                                                                                                    as
                                                                                                                    libc::c_uint)
                      as *mut DMA_Stream_TypeDef)).CR &
                   (0x1 as libc::c_uint) << 18 as libc::c_uint !=
                   0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).HIFCR
                                                as *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                21 as libc::c_uint)
                // Clear the transfer complete flag
            } else {
                //Disable the transfer complete interrupt if the DMA mode is not CIRCULAR
                if (*((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x6400
                                                                                         as
                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                        as
                                                                                                                        libc::c_uint)
                          as *mut DMA_Stream_TypeDef)).CR &
                       (0x1 as libc::c_uint) << 8 as libc::c_uint ==
                       0 as libc::c_int as libc::c_uint {
                    let ref mut fresh29 =
                        (*((0x40000000 as
                                libc::c_uint).wrapping_add(0x20000 as
                                                               libc::c_uint).wrapping_add(0x6400
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0xa0
                                                                                                                             as
                                                                                                                             libc::c_uint)
                               as *mut DMA_Stream_TypeDef)).CR;
                    ::core::ptr::write_volatile(fresh29,
                                                (::core::ptr::read_volatile::<uint32_t>(fresh29
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           4 as libc::c_uint))
                                                    as uint32_t as uint32_t)
                    // Disable the transfer complete interrupt
                } // Clear the transfer complete flag
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).HIFCR
                                                as *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                21 as libc::c_uint);
                SD_DMA_Complete((0x40000000 as
                                     libc::c_uint).wrapping_add(0x20000 as
                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                   as
                                                                                                   libc::c_uint).wrapping_add(0xa0
                                                                                                                                  as
                                                                                                                                  libc::c_uint)
                                    as *mut DMA_Stream_TypeDef);
            }
        }
    };
}
/* ------------------------------------------------------------------------------------------------------------------*/
