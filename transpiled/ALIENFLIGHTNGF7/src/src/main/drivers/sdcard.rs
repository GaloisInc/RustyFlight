use ::libc;
extern "C" {
    /* *
  * @}
  */
    /* * @defgroup DMA_LL_EF_Init Initialization and de-initialization functions
  * @{
  */
    #[no_mangle]
    fn LL_DMA_Init(DMAx: *mut DMA_TypeDef, Stream: uint32_t,
                   DMA_InitStruct: *mut LL_DMA_InitTypeDef) -> uint32_t;
    #[no_mangle]
    fn LL_DMA_DeInit(DMAx: *mut DMA_TypeDef, Stream: uint32_t) -> uint32_t;
    #[no_mangle]
    fn LL_DMA_StructInit(DMA_InitStruct: *mut LL_DMA_InitTypeDef);
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn dmaGetChannel(channel: uint8_t) -> uint32_t;
    #[no_mangle]
    fn dmaInit(identifier: dmaIdentifier_e, owner: resourceOwner_e,
               resourceIndex: uint8_t);
    #[no_mangle]
    fn dmaGetDescriptorByIdentifier(identifier: dmaIdentifier_e)
     -> *mut dmaChannelDescriptor_t;
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiTransferByte(instance: *mut SPI_TypeDef, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn spiIsBusBusy(instance: *mut SPI_TypeDef) -> bool;
    #[no_mangle]
    fn spiTransfer(instance: *mut SPI_TypeDef, txData: *const uint8_t,
                   rxData: *mut uint8_t, len: libc::c_int) -> bool;
    #[no_mangle]
    fn spiInstanceByDevice(device: SPIDevice) -> *mut SPI_TypeDef;
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
    fn millis() -> timeMs_t;
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn readBitfield(buffer: *mut uint8_t, bitIndex: libc::c_uint,
                    bitLen: libc::c_uint) -> uint32_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
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
pub struct SPI_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SR: uint32_t,
    pub DR: uint32_t,
    pub CRCPR: uint32_t,
    pub RXCRCR: uint32_t,
    pub TXCRCR: uint32_t,
    pub I2SCFGR: uint32_t,
    pub I2SPR: uint32_t,
}
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
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
pub type dmaCallbackHandlerFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut dmaChannelDescriptor_s) -> ()>;
pub type dmaChannelDescriptor_t = dmaChannelDescriptor_s;
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
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
// millisecond time
pub type timeMs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardConfig_s {
    pub useDma: uint8_t,
    pub enabled: uint8_t,
    pub device: uint8_t,
    pub cardDetectTag: ioTag_t,
    pub chipSelectTag: ioTag_t,
    pub cardDetectInverted: uint8_t,
    pub dmaIdentifier: uint8_t,
    pub dmaChannel: uint8_t,
}
pub type sdcardConfig_t = sdcardConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardMetadata_s {
    pub numBlocks: uint32_t,
    pub oemID: uint16_t,
    pub manufacturerID: uint8_t,
    pub productName: [libc::c_char; 5],
    pub productSerial: uint32_t,
    pub productRevisionMajor: uint8_t,
    pub productRevisionMinor: uint8_t,
    pub productionYear: uint16_t,
    pub productionMonth: uint8_t,
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
pub type sdcardMetadata_t = sdcardMetadata_s;
pub type sdcardBlockOperation_e = libc::c_uint;
pub const SDCARD_BLOCK_OPERATION_ERASE: sdcardBlockOperation_e = 2;
pub const SDCARD_BLOCK_OPERATION_WRITE: sdcardBlockOperation_e = 1;
pub const SDCARD_BLOCK_OPERATION_READ: sdcardBlockOperation_e = 0;
pub type sdcardOperationStatus_e = libc::c_uint;
pub const SDCARD_OPERATION_FAILURE: sdcardOperationStatus_e = 3;
pub const SDCARD_OPERATION_SUCCESS: sdcardOperationStatus_e = 2;
pub const SDCARD_OPERATION_BUSY: sdcardOperationStatus_e = 1;
pub const SDCARD_OPERATION_IN_PROGRESS: sdcardOperationStatus_e = 0;
pub type sdcard_operationCompleteCallback_c
    =
    Option<unsafe extern "C" fn(_: sdcardBlockOperation_e, _: uint32_t,
                                _: *mut uint8_t, _: uint32_t) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcard_t {
    pub pendingOperation: C2RustUnnamed,
    pub operationStartTime: uint32_t,
    pub failureCount: uint8_t,
    pub version: uint8_t,
    pub highCapacity: bool,
    pub multiWriteNextBlock: uint32_t,
    pub multiWriteBlocksRemain: uint32_t,
    pub state: sdcardState_e,
    pub metadata: sdcardMetadata_t,
    pub csd: sdcardCSD_t,
    pub instance: *mut SPI_TypeDef,
    pub enabled: bool,
    pub detectionInverted: bool,
    pub useDMAForTx: bool,
    pub cardDetectPin: IO_t,
    pub chipSelectPin: IO_t,
    pub dma: *mut dmaChannelDescriptor_t,
    pub dmaChannel: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardCSD_t {
    pub data: [uint8_t; 16],
}
pub type sdcardState_e = libc::c_uint;
pub const SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE: sdcardState_e = 9;
pub const SDCARD_STATE_WRITING_MULTIPLE_BLOCKS: sdcardState_e = 8;
pub const SDCARD_STATE_WAITING_FOR_WRITE: sdcardState_e = 7;
pub const SDCARD_STATE_SENDING_WRITE: sdcardState_e = 6;
pub const SDCARD_STATE_READING: sdcardState_e = 5;
pub const SDCARD_STATE_READY: sdcardState_e = 4;
pub const SDCARD_STATE_INITIALIZATION_RECEIVE_CID: sdcardState_e = 3;
pub const SDCARD_STATE_CARD_INIT_IN_PROGRESS: sdcardState_e = 2;
pub const SDCARD_STATE_RESET: sdcardState_e = 1;
pub const SDCARD_STATE_NOT_PRESENT: sdcardState_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed {
    pub buffer: *mut uint8_t,
    pub blockIndex: uint32_t,
    pub chunkIndex: uint8_t,
    pub callback: sdcard_operationCompleteCallback_c,
    pub callbackData: uint32_t,
}
pub const SDCARD_RECEIVE_ERROR: sdcardReceiveBlockStatus_e = 2;
pub const SDCARD_RECEIVE_BLOCK_IN_PROGRESS: sdcardReceiveBlockStatus_e = 1;
pub const SDCARD_RECEIVE_SUCCESS: sdcardReceiveBlockStatus_e = 0;
pub type sdcardReceiveBlockStatus_e = libc::c_uint;
/* Card capacity in 512-byte blocks*/
/* *
  * @}
  */
/* * @defgroup SPI_LL_EF_FLAG_Management FLAG Management
  * @{
  */
/* *
  * @brief  Check if Rx buffer is not empty
  * @rmtoll SR           RXNE          LL_SPI_IsActiveFlag_RXNE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
#[inline]
unsafe extern "C" fn LL_SPI_IsActiveFlag_RXNE(mut SPIx: *mut SPI_TypeDef)
 -> uint32_t {
    return ((*SPIx).SR & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                (0x1 as libc::c_uint) << 0 as libc::c_uint) as libc::c_int as
               uint32_t;
}
/* *
  * @brief  Enable DMA Tx
  * @rmtoll CR2          TXDMAEN       LL_SPI_EnableDMAReq_TX
  * @param  SPIx SPI Instance
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_SPI_EnableDMAReq_TX(mut SPIx: *mut SPI_TypeDef) {
    ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*SPIx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Disable DMA Tx
  * @rmtoll CR2          TXDMAEN       LL_SPI_DisableDMAReq_TX
  * @param  SPIx SPI Instance
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_SPI_DisableDMAReq_TX(mut SPIx: *mut SPI_TypeDef) {
    ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*SPIx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
// Initialized in run_static_initializers
static mut STREAM_OFFSET_TAB: [uint8_t; 8] = [0; 8];
#[inline]
unsafe extern "C" fn LL_DMA_EnableStream(mut DMAx: *mut DMA_TypeDef,
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
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
static mut sdcard: sdcard_t =
    sdcard_t{pendingOperation:
                 C2RustUnnamed{buffer: 0 as *const uint8_t as *mut uint8_t,
                               blockIndex: 0,
                               chunkIndex: 0,
                               callback: None,
                               callbackData: 0,},
             operationStartTime: 0,
             failureCount: 0,
             version: 0,
             highCapacity: false,
             multiWriteNextBlock: 0,
             multiWriteBlocksRemain: 0,
             state: SDCARD_STATE_NOT_PRESENT,
             metadata:
                 sdcardMetadata_t{numBlocks: 0,
                                  oemID: 0,
                                  manufacturerID: 0,
                                  productName: [0; 5],
                                  productSerial: 0,
                                  productRevisionMajor: 0,
                                  productRevisionMinor: 0,
                                  productionYear: 0,
                                  productionMonth: 0,},
             csd: sdcardCSD_t{data: [0; 16],},
             instance: 0 as *const SPI_TypeDef as *mut SPI_TypeDef,
             enabled: false,
             detectionInverted: false,
             useDMAForTx: false,
             cardDetectPin: 0 as *const libc::c_void as *mut libc::c_void,
             chipSelectPin: 0 as *const libc::c_void as *mut libc::c_void,
             dma:
                 0 as *const dmaChannelDescriptor_t as
                     *mut dmaChannelDescriptor_t,
             dmaChannel: 0,};
#[no_mangle]
pub unsafe extern "C" fn sdcardInsertionDetectDeinit() {
    if !sdcard.cardDetectPin.is_null() {
        IOInit(sdcard.cardDetectPin, OWNER_FREE, 0 as libc::c_int as uint8_t);
        IOConfigGPIO(sdcard.cardDetectPin,
                     (0 as libc::c_uint |
                          (0 as libc::c_uint) << 2 as libc::c_int |
                          (0 as libc::c_uint) << 5 as libc::c_int) as
                         ioConfig_t);
    };
}
#[no_mangle]
pub unsafe extern "C" fn sdcardInsertionDetectInit() {
    if !sdcard.cardDetectPin.is_null() {
        IOInit(sdcard.cardDetectPin, OWNER_SDCARD_DETECT,
               0 as libc::c_int as uint8_t);
        IOConfigGPIO(sdcard.cardDetectPin,
                     (0 as libc::c_uint |
                          (0 as libc::c_uint) << 2 as libc::c_int |
                          (0x1 as libc::c_uint) << 5 as libc::c_int) as
                         ioConfig_t);
    };
}
/* *
 * Detect if a SD card is physically present in the memory slot.
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_isInserted() -> bool {
    let mut result: bool = 1 as libc::c_int != 0;
    if !sdcard.cardDetectPin.is_null() {
        result =
            IORead(sdcard.cardDetectPin) as libc::c_int != 0 as libc::c_int;
        if sdcard.detectionInverted { result = !result }
    }
    return result;
}
/* *
 * Returns true if the card has already been, or is currently, initializing and hasn't encountered enough errors to
 * trip our error threshold and be disabled (i.e. our card is in and working!)
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_isFunctional() -> bool {
    return sdcard.state as libc::c_uint !=
               SDCARD_STATE_NOT_PRESENT as libc::c_int as libc::c_uint;
}
unsafe extern "C" fn sdcard_select() { IOLo(sdcard.chipSelectPin); }
unsafe extern "C" fn sdcard_deselect() {
    // As per the SD-card spec, give the card 8 dummy clocks so it can finish its operation
    //spiTransferByte(sdcard.instance, 0xFF);
    while spiIsBusBusy(sdcard.instance) { }
    IOHi(sdcard.chipSelectPin);
}
/* *
 * Handle a failure of an SD card operation by resetting the card back to its initialization phase.
 *
 * Increments the failure counter, and when the failure threshold is reached, disables the card until
 * the next call to sdcard_init().
 */
unsafe extern "C" fn sdcard_reset() {
    if !sdcard_isInserted() {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
        return
    }
    if sdcard.state as libc::c_uint >=
           SDCARD_STATE_READY as libc::c_int as libc::c_uint {
        spiSetDivisor(sdcard.instance, 256 as libc::c_int as uint16_t);
    }
    sdcard.failureCount = sdcard.failureCount.wrapping_add(1);
    if sdcard.failureCount as libc::c_int >= 8 as libc::c_int {
        sdcard.state = SDCARD_STATE_NOT_PRESENT
    } else {
        sdcard.operationStartTime = millis();
        sdcard.state = SDCARD_STATE_RESET
    };
}
/* *
 * The SD card spec requires 8 clock cycles to be sent by us on the bus after most commands so it can finish its
 * processing of that command. The easiest way for us to do this is to just wait for the bus to become idle before
 * we transmit a command, sending at least 8-bits onto the bus when we do so.
 */
unsafe extern "C" fn sdcard_waitForIdle(mut maxBytesToWait: libc::c_int)
 -> bool {
    while maxBytesToWait > 0 as libc::c_int {
        let mut b: uint8_t =
            spiTransferByte(sdcard.instance, 0xff as libc::c_int as uint8_t);
        if b as libc::c_int == 0xff as libc::c_int {
            return 1 as libc::c_int != 0
        }
        maxBytesToWait -= 1
    }
    return 0 as libc::c_int != 0;
}
/* *
 * Wait for up to maxDelay 0xFF idle bytes to arrive from the card, returning the first non-idle byte found.
 *
 * Returns 0xFF on failure.
 */
unsafe extern "C" fn sdcard_waitForNonIdleByte(mut maxDelay: libc::c_int)
 -> uint8_t {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < maxDelay + 1 as libc::c_int {
        // + 1 so we can wait for maxDelay '0xFF' bytes before reading a response byte afterwards
        let mut response: uint8_t =
            spiTransferByte(sdcard.instance, 0xff as libc::c_int as uint8_t);
        if response as libc::c_int != 0xff as libc::c_int { return response }
        i += 1
    }
    return 0xff as libc::c_int as uint8_t;
}
/* *
 * Waits up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes for the card to become ready, send a command to the card
 * with the given argument, waits up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes for a reply, and returns the
 * first non-0xFF byte of the reply.
 *
 * You must select the card first with sdcard_select() and deselect it afterwards with sdcard_deselect().
 *
 * Upon failure, 0xFF is returned.
 */
unsafe extern "C" fn sdcard_sendCommand(mut commandCode: uint8_t,
                                        mut commandArgument: uint32_t)
 -> uint8_t {
    let command: [uint8_t; 6] =
        [(0x40 as libc::c_int | commandCode as libc::c_int) as uint8_t,
         (commandArgument >> 24 as libc::c_int) as uint8_t,
         (commandArgument >> 16 as libc::c_int) as uint8_t,
         (commandArgument >> 8 as libc::c_int) as uint8_t,
         commandArgument as uint8_t, 0x95 as libc::c_int as uint8_t];
    // Go ahead and send the command even if the card isn't idle if this is the reset command
    if !sdcard_waitForIdle(8 as libc::c_int) &&
           commandCode as libc::c_int != 0 as libc::c_int {
        return 0xff as libc::c_int as uint8_t
    }
    spiTransfer(sdcard.instance, command.as_ptr(), 0 as *mut uint8_t,
                ::core::mem::size_of::<[uint8_t; 6]>() as libc::c_ulong as
                    libc::c_int);
    /*
     * The card can take up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes to send the response, in the meantime
     * it'll transmit 0xFF filler bytes.
     */
    return sdcard_waitForNonIdleByte(8 as libc::c_int);
}
unsafe extern "C" fn sdcard_sendAppCommand(mut commandCode: uint8_t,
                                           mut commandArgument: uint32_t)
 -> uint8_t {
    sdcard_sendCommand(55 as libc::c_int as uint8_t,
                       0 as libc::c_int as uint32_t);
    return sdcard_sendCommand(commandCode, commandArgument);
}
/* *
 * Sends an IF_COND message to the card to check its version and validate its voltage requirements. Sets the global
 * sdCardVersion with the detected version (0, 1, or 2) and returns true if the card is compatible.
 */
unsafe extern "C" fn sdcard_validateInterfaceCondition() -> bool {
    let mut ifCondReply: [uint8_t; 4] = [0; 4];
    sdcard.version = 0 as libc::c_int as uint8_t;
    sdcard_select();
    let mut status: uint8_t =
        sdcard_sendCommand(8 as libc::c_int as uint8_t,
                           ((0x1 as libc::c_int) << 8 as libc::c_int |
                                0xab as libc::c_int) as uint32_t);
    // Don't deselect the card right away, because we'll want to read the rest of its reply if it's a V2 card
    if status as libc::c_int == 4 as libc::c_int | 1 as libc::c_int {
        // V1 cards don't support this command
        sdcard.version = 1 as libc::c_int as uint8_t
    } else if status as libc::c_int == 1 as libc::c_int {
        spiTransfer(sdcard.instance, 0 as *const uint8_t,
                    ifCondReply.as_mut_ptr(),
                    ::core::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong as
                        libc::c_int);
        /*
         * We don't bother to validate the SDCard's operating voltage range since the spec requires it to accept our
         * 3.3V, but do check that it echoed back our check pattern properly.
         */
        if ifCondReply[3 as libc::c_int as usize] as libc::c_int ==
               0xab as libc::c_int {
            sdcard.version = 2 as libc::c_int as uint8_t
        }
    }
    sdcard_deselect();
    return sdcard.version as libc::c_int > 0 as libc::c_int;
}
unsafe extern "C" fn sdcard_readOCRRegister(mut result: *mut uint32_t)
 -> bool {
    sdcard_select();
    let mut status: uint8_t =
        sdcard_sendCommand(58 as libc::c_int as uint8_t,
                           0 as libc::c_int as uint32_t);
    let mut response: [uint8_t; 4] = [0; 4];
    spiTransfer(sdcard.instance, 0 as *const uint8_t, response.as_mut_ptr(),
                ::core::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong as
                    libc::c_int);
    if status as libc::c_int == 0 as libc::c_int {
        sdcard_deselect();
        *result =
            ((response[0 as libc::c_int as usize] as libc::c_int) <<
                 24 as libc::c_int |
                 (response[1 as libc::c_int as usize] as libc::c_int) <<
                     16 as libc::c_int |
                 (response[2 as libc::c_int as usize] as libc::c_int) <<
                     8 as libc::c_int |
                 response[3 as libc::c_int as usize] as libc::c_int) as
                uint32_t;
        return 1 as libc::c_int != 0
    } else { sdcard_deselect(); return 0 as libc::c_int != 0 };
}
/* *
 * Attempt to receive a data block from the SD card.
 *
 * Return true on success, otherwise the card has not responded yet and you should retry later.
 */
unsafe extern "C" fn sdcard_receiveDataBlock(mut buffer: *mut uint8_t,
                                             mut count: libc::c_int)
 -> sdcardReceiveBlockStatus_e {
    let mut dataToken: uint8_t = sdcard_waitForNonIdleByte(8 as libc::c_int);
    if dataToken as libc::c_int == 0xff as libc::c_int {
        return SDCARD_RECEIVE_BLOCK_IN_PROGRESS
    }
    if dataToken as libc::c_int != 0xfe as libc::c_int {
        return SDCARD_RECEIVE_ERROR
    }
    spiTransfer(sdcard.instance, 0 as *const uint8_t, buffer, count);
    // Discard trailing CRC, we don't care
    spiTransferByte(sdcard.instance, 0xff as libc::c_int as uint8_t);
    spiTransferByte(sdcard.instance, 0xff as libc::c_int as uint8_t);
    return SDCARD_RECEIVE_SUCCESS;
}
unsafe extern "C" fn sdcard_sendDataBlockFinish() -> bool {
    // Drain anything left in the Rx FIFO (we didn't read it during the write)
    //This is necessary here as when using msc there is timing issue
    while LL_SPI_IsActiveFlag_RXNE(sdcard.instance) != 0 { }
    // Send a dummy CRC
    spiTransferByte(sdcard.instance, 0 as libc::c_int as uint8_t);
    spiTransferByte(sdcard.instance, 0 as libc::c_int as uint8_t);
    let mut dataResponseToken: uint8_t =
        spiTransferByte(sdcard.instance, 0xff as libc::c_int as uint8_t);
    /*
     * Check if the card accepted the write (no CRC error / no address error)
     *
     * The lower 5 bits are structured as follows:
     * | 0 | Status  | 1 |
     * | 0 | x  x  x | 1 |
     *
     * Statuses:
     * 010 - Data accepted
     * 101 - CRC error
     * 110 - Write error
     */
    return dataResponseToken as libc::c_int & 0x1f as libc::c_int ==
               0x5 as libc::c_int;
}
/* *
 * Begin sending a buffer of SDCARD_BLOCK_SIZE bytes to the SD card.
 */
unsafe extern "C" fn sdcard_sendDataBlockBegin(mut buffer: *const uint8_t,
                                               mut multiBlockWrite: bool) {
    // Card wants 8 dummy clock cycles between the write command's response and a data block beginning:
    spiTransferByte(sdcard.instance, 0xff as libc::c_int as uint8_t);
    spiTransferByte(sdcard.instance,
                    if multiBlockWrite as libc::c_int != 0 {
                        0xfc as libc::c_int
                    } else { 0xfe as libc::c_int } as uint8_t);
    if sdcard.useDMAForTx {
        let mut init: LL_DMA_InitTypeDef =
            LL_DMA_InitTypeDef{PeriphOrM2MSrcAddress: 0,
                               MemoryOrM2MDstAddress: 0,
                               Direction: 0,
                               Mode: 0,
                               PeriphOrM2MSrcIncMode: 0,
                               MemoryOrM2MDstIncMode: 0,
                               PeriphOrM2MSrcDataSize: 0,
                               MemoryOrM2MDstDataSize: 0,
                               NbData: 0,
                               Channel: 0,
                               Priority: 0,
                               FIFOMode: 0,
                               FIFOThreshold: 0,
                               MemBurst: 0,
                               PeriphBurst: 0,};
        LL_DMA_StructInit(&mut init);
        init.Channel = dmaGetChannel(sdcard.dmaChannel);
        init.Mode = 0 as libc::c_uint;
        init.Direction = (0x1 as libc::c_uint) << 6 as libc::c_uint;
        init.PeriphOrM2MSrcAddress =
            &mut (*sdcard.instance).DR as *mut uint32_t as uint32_t;
        init.Priority = 0 as libc::c_uint;
        init.PeriphOrM2MSrcIncMode = 0 as libc::c_uint;
        init.PeriphOrM2MSrcDataSize = 0 as libc::c_uint;
        init.MemoryOrM2MDstAddress = buffer as uint32_t;
        init.MemoryOrM2MDstIncMode =
            (0x1 as libc::c_uint) << 10 as libc::c_uint;
        init.MemoryOrM2MDstDataSize = 0 as libc::c_uint;
        init.NbData = 512 as libc::c_int as uint32_t;
        LL_DMA_DeInit((*sdcard.dma).dma, (*sdcard.dma).stream as uint32_t);
        LL_DMA_Init((*sdcard.dma).dma, (*sdcard.dma).stream as uint32_t,
                    &mut init);
        LL_DMA_EnableStream((*sdcard.dma).dma,
                            (*sdcard.dma).stream as uint32_t);
        LL_SPI_EnableDMAReq_TX(sdcard.instance);
    } else {
        // Send the first chunk now
        spiTransfer(sdcard.instance, buffer, 0 as *mut uint8_t,
                    256 as libc::c_int);
    };
}
unsafe extern "C" fn sdcard_receiveCID() -> bool {
    let mut cid: [uint8_t; 16] = [0; 16];
    if sdcard_receiveDataBlock(cid.as_mut_ptr(),
                               ::core::mem::size_of::<[uint8_t; 16]>() as
                                   libc::c_ulong as libc::c_int) as
           libc::c_uint !=
           SDCARD_RECEIVE_SUCCESS as libc::c_int as libc::c_uint {
        return 0 as libc::c_int != 0
    }
    sdcard.metadata.manufacturerID = cid[0 as libc::c_int as usize];
    sdcard.metadata.oemID =
        ((cid[1 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int |
             cid[2 as libc::c_int as usize] as libc::c_int) as uint16_t;
    sdcard.metadata.productName[0 as libc::c_int as usize] =
        cid[3 as libc::c_int as usize] as libc::c_char;
    sdcard.metadata.productName[1 as libc::c_int as usize] =
        cid[4 as libc::c_int as usize] as libc::c_char;
    sdcard.metadata.productName[2 as libc::c_int as usize] =
        cid[5 as libc::c_int as usize] as libc::c_char;
    sdcard.metadata.productName[3 as libc::c_int as usize] =
        cid[6 as libc::c_int as usize] as libc::c_char;
    sdcard.metadata.productName[4 as libc::c_int as usize] =
        cid[7 as libc::c_int as usize] as libc::c_char;
    sdcard.metadata.productRevisionMajor =
        (cid[8 as libc::c_int as usize] as libc::c_int >> 4 as libc::c_int) as
            uint8_t;
    sdcard.metadata.productRevisionMinor =
        (cid[8 as libc::c_int as usize] as libc::c_int & 0xf as libc::c_int)
            as uint8_t;
    sdcard.metadata.productSerial =
        ((cid[9 as libc::c_int as usize] as libc::c_int) << 24 as libc::c_int
             |
             (cid[10 as libc::c_int as usize] as libc::c_int) <<
                 16 as libc::c_int |
             (cid[11 as libc::c_int as usize] as libc::c_int) <<
                 8 as libc::c_int |
             cid[12 as libc::c_int as usize] as libc::c_int) as uint32_t;
    sdcard.metadata.productionYear =
        (((cid[13 as libc::c_int as usize] as libc::c_int &
               0xf as libc::c_int) << 4 as libc::c_int |
              cid[14 as libc::c_int as usize] as libc::c_int >>
                  4 as libc::c_int) + 2000 as libc::c_int) as uint16_t;
    sdcard.metadata.productionMonth =
        (cid[14 as libc::c_int as usize] as libc::c_int & 0xf as libc::c_int)
            as uint8_t;
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn sdcard_fetchCSD() -> bool {
    let mut readBlockLen: uint32_t = 0;
    let mut blockCount: uint32_t = 0;
    let mut blockCountMult: uint32_t = 0;
    let mut capacityBytes: uint64_t = 0;
    sdcard_select();
    /* The CSD command's data block should always arrive within 8 idle clock cycles (SD card spec). This is because
     * the information about card latency is stored in the CSD register itself, so we can't use that yet!
     */
    let mut success: bool =
        sdcard_sendCommand(9 as libc::c_int as uint8_t,
                           0 as libc::c_int as uint32_t) as libc::c_int ==
            0 as libc::c_int &&
            sdcard_receiveDataBlock(&mut sdcard.csd as *mut sdcardCSD_t as
                                        *mut uint8_t,
                                    ::core::mem::size_of::<sdcardCSD_t>() as
                                        libc::c_ulong as libc::c_int) as
                libc::c_uint ==
                SDCARD_RECEIVE_SUCCESS as libc::c_int as libc::c_uint &&
            readBitfield(sdcard.csd.data.as_mut_ptr(),
                         127 as libc::c_int as libc::c_uint,
                         1 as libc::c_int as libc::c_uint) ==
                1 as libc::c_int as libc::c_uint;
    if success {
        match readBitfield(sdcard.csd.data.as_mut_ptr(),
                           0 as libc::c_int as libc::c_uint,
                           2 as libc::c_int as libc::c_uint) {
            0 => {
                // Block size in bytes (doesn't have to be 512)
                readBlockLen =
                    ((1 as libc::c_int) <<
                         readBitfield(sdcard.csd.data.as_mut_ptr(),
                                      44 as libc::c_int as libc::c_uint,
                                      4 as libc::c_int as libc::c_uint)) as
                        uint32_t;
                blockCountMult =
                    ((1 as libc::c_int) <<
                         readBitfield(sdcard.csd.data.as_mut_ptr(),
                                      78 as libc::c_int as libc::c_uint,
                                      3 as libc::c_int as
                                          libc::c_uint).wrapping_add(2 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint))
                        as uint32_t;
                blockCount =
                    readBitfield(sdcard.csd.data.as_mut_ptr(),
                                 54 as libc::c_int as libc::c_uint,
                                 12 as libc::c_int as
                                     libc::c_uint).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint).wrapping_mul(blockCountMult);
                // We could do this in 32 bits but it makes the 2GB case awkward
                capacityBytes =
                    (blockCount as
                         uint64_t).wrapping_mul(readBlockLen as
                                                    libc::c_ulong);
                // Re-express that capacity (max 2GB) in our standard 512-byte block size
                sdcard.metadata.numBlocks =
                    capacityBytes.wrapping_div(512 as libc::c_int as
                                                   libc::c_ulong) as uint32_t
            }
            1 => {
                sdcard.metadata.numBlocks =
                    readBitfield(sdcard.csd.data.as_mut_ptr(),
                                 58 as libc::c_int as libc::c_uint,
                                 22 as libc::c_int as
                                     libc::c_uint).wrapping_add(1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint).wrapping_mul(1024
                                                                                                   as
                                                                                                   libc::c_int
                                                                                                   as
                                                                                                   libc::c_uint)
            }
            _ => { success = 0 as libc::c_int != 0 }
        }
    }
    sdcard_deselect();
    return success;
}
/* *
 * Check if the SD Card has completed its startup sequence. Must be called with sdcard.state == SDCARD_STATE_INITIALIZATION.
 *
 * Returns true if the card has finished its init process.
 */
unsafe extern "C" fn sdcard_checkInitDone() -> bool {
    sdcard_select();
    let mut status: uint8_t =
        sdcard_sendAppCommand(41 as libc::c_int as uint8_t,
                              if sdcard.version as libc::c_int ==
                                     2 as libc::c_int {
                                  ((1 as libc::c_int)) << 30 as libc::c_int
                              } else { 0 as libc::c_int } as uint32_t);
    sdcard_deselect();
    // When card init is complete, the idle bit in the response becomes zero.
    return status as libc::c_int == 0 as libc::c_int;
}
/* *
 * Begin the initialization process for the SD card. This must be called first before any other sdcard_ routine.
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_init(mut config: *const sdcardConfig_t) {
    sdcard.enabled = (*config).enabled != 0;
    if !sdcard.enabled { sdcard.state = SDCARD_STATE_NOT_PRESENT; return }
    sdcard.instance = spiInstanceByDevice((*config).device as SPIDevice);
    sdcard.useDMAForTx = (*config).useDma != 0;
    if sdcard.useDMAForTx {
        sdcard.dmaChannel = (*config).dmaChannel;
        sdcard.dma =
            dmaGetDescriptorByIdentifier((*config).dmaIdentifier as
                                             dmaIdentifier_e);
        dmaInit((*config).dmaIdentifier as dmaIdentifier_e, OWNER_SDCARD,
                0 as libc::c_int as uint8_t);
    }
    if (*config).chipSelectTag != 0 {
        sdcard.chipSelectPin = IOGetByTag((*config).chipSelectTag);
        IOInit(sdcard.chipSelectPin, OWNER_SDCARD_CS,
               0 as libc::c_int as uint8_t);
        IOConfigGPIO(sdcard.chipSelectPin,
                     (0x1 as libc::c_uint |
                          (0x3 as libc::c_uint) << 2 as libc::c_int |
                          (0 as libc::c_uint) << 5 as libc::c_int) as
                         ioConfig_t);
    } else { sdcard.chipSelectPin = 0 as IO_t }
    if (*config).cardDetectTag != 0 {
        sdcard.cardDetectPin = IOGetByTag((*config).cardDetectTag);
        sdcard.detectionInverted = (*config).cardDetectInverted != 0
    } else {
        sdcard.cardDetectPin = 0 as IO_t;
        sdcard.detectionInverted = 0 as libc::c_int != 0
    }
    // Max frequency is initially 400kHz
    spiSetDivisor(sdcard.instance, 256 as libc::c_int as uint16_t);
    // SDCard wants 1ms minimum delay after power is applied to it
    delay(1000 as libc::c_int as timeMs_t);
    // Transmit at least 74 dummy clock cycles with CS high so the SD card can start up
    IOHi(sdcard.chipSelectPin);
    spiTransfer(sdcard.instance, 0 as *const uint8_t, 0 as *mut uint8_t,
                10 as libc::c_int);
    // Wait for that transmission to finish before we enable the SDCard, so it receives the required number of cycles:
    let mut time: libc::c_int = 100000 as libc::c_int;
    while spiIsBusBusy(sdcard.instance) {
        let fresh1 = time;
        time = time - 1;
        if fresh1 == 0 as libc::c_int {
            sdcard.state = SDCARD_STATE_NOT_PRESENT;
            sdcard.failureCount = sdcard.failureCount.wrapping_add(1);
            return
        }
    }
    sdcard.operationStartTime = millis();
    sdcard.state = SDCARD_STATE_RESET;
    sdcard.failureCount = 0 as libc::c_int as uint8_t;
}
unsafe extern "C" fn sdcard_setBlockLength(mut blockLen: uint32_t) -> bool {
    sdcard_select();
    let mut status: uint8_t =
        sdcard_sendCommand(16 as libc::c_int as uint8_t, blockLen);
    sdcard_deselect();
    return status as libc::c_int == 0 as libc::c_int;
}
/*
 * Returns true if the card is ready to accept read/write commands.
 */
unsafe extern "C" fn sdcard_isReady() -> bool {
    return sdcard.state as libc::c_uint ==
               SDCARD_STATE_READY as libc::c_int as libc::c_uint ||
               sdcard.state as libc::c_uint ==
                   SDCARD_STATE_WRITING_MULTIPLE_BLOCKS as libc::c_int as
                       libc::c_uint;
}
/* *
 * Send the stop-transmission token to complete a multi-block write.
 *
 * Returns:
 *     SDCARD_OPERATION_IN_PROGRESS - We're now waiting for that stop to complete, the card will enter
 *                                    the SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE state.
 *     SDCARD_OPERATION_SUCCESS     - The multi-block write finished immediately, the card will enter
 *                                    the SDCARD_READY state.
 *
 */
unsafe extern "C" fn sdcard_endWriteBlocks() -> sdcardOperationStatus_e {
    sdcard.multiWriteBlocksRemain = 0 as libc::c_int as uint32_t;
    // 8 dummy clocks to guarantee N_WR clocks between the last card response and this token
    spiTransferByte(sdcard.instance, 0xff as libc::c_int as uint8_t);
    spiTransferByte(sdcard.instance, 0xfd as libc::c_int as uint8_t);
    // Card may choose to raise a busy (non-0xFF) signal after at most N_BR (1 byte) delay
    if sdcard_waitForNonIdleByte(1 as libc::c_int) as libc::c_int ==
           0xff as libc::c_int {
        sdcard.state = SDCARD_STATE_READY;
        return SDCARD_OPERATION_SUCCESS
    } else {
        sdcard.state = SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE;
        sdcard.operationStartTime = millis();
        return SDCARD_OPERATION_IN_PROGRESS
    };
}
/* *
 * Call periodically for the SD card to perform in-progress transfers.
 *
 * Returns true if the card is ready to accept commands.
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_poll() -> bool {
    if !sdcard.enabled {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
        return 0 as libc::c_int != 0
    }
    let mut initStatus: uint8_t = 0;
    let mut sendComplete: bool = false;
    loop  {
        match sdcard.state as libc::c_uint {
            1 => {
                sdcard_select();
                initStatus =
                    sdcard_sendCommand(0 as libc::c_int as uint8_t,
                                       0 as libc::c_int as uint32_t);
                sdcard_deselect();
                if !(initStatus as libc::c_int == 1 as libc::c_int) {
                    break ;
                }
                // Check card voltage and version
                if sdcard_validateInterfaceCondition() {
                    sdcard.state = SDCARD_STATE_CARD_INIT_IN_PROGRESS
                } else {
                    // Bad reply/voltage, we ought to refrain from accessing the card.
                    sdcard.state = SDCARD_STATE_NOT_PRESENT;
                    break ;
                }
            }
            2 => {
                if !sdcard_checkInitDone() { break ; }
                if sdcard.version as libc::c_int == 2 as libc::c_int {
                    // Check for high capacity card
                    let mut ocr: uint32_t = 0;
                    if !sdcard_readOCRRegister(&mut ocr) {
                        sdcard_reset();
                        continue ;
                    } else {
                        sdcard.highCapacity =
                            ocr &
                                ((1 as libc::c_int) << 30 as libc::c_int) as
                                    libc::c_uint !=
                                0 as libc::c_int as libc::c_uint
                    }
                } else {
                    // Version 1 cards are always low-capacity
                    sdcard.highCapacity = 0 as libc::c_int != 0
                }
                // Now fetch the CSD and CID registers
                if !sdcard_fetchCSD() { break ; }
                sdcard_select();
                let mut status: uint8_t =
                    sdcard_sendCommand(10 as libc::c_int as uint8_t,
                                       0 as libc::c_int as uint32_t);
                if status as libc::c_int == 0 as libc::c_int {
                    // Keep the card selected to receive the response block
                    sdcard.state = SDCARD_STATE_INITIALIZATION_RECEIVE_CID
                } else {
                    sdcard_deselect(); // else keep waiting for the CID to arrive
                    sdcard_reset();
                }
            }
            3 => {
                if !sdcard_receiveCID() { break ; }
                sdcard_deselect();
                /* The spec is a little iffy on what the default block size is for Standard Size cards (it can be changed on
                 * standard size cards) so let's just set it to 512 explicitly so we don't have a problem.
                 */
                if !sdcard.highCapacity &&
                       !sdcard_setBlockLength(512 as libc::c_int as uint32_t)
                   {
                    sdcard_reset();
                } else {
                    // Now we're done with init and we can switch to the full speed clock (<25MHz)
                    spiSetDivisor(sdcard.instance,
                                  8 as libc::c_int as uint16_t);
                    sdcard.multiWriteBlocksRemain =
                        0 as libc::c_int as uint32_t;
                    sdcard.state = SDCARD_STATE_READY
                }
            }
            6 => {
                // Have we finished sending the write yet?
                sendComplete = 0 as libc::c_int != 0;
                if sdcard.useDMAForTx as libc::c_int != 0 &&
                       (if (*sdcard.dma).flagsShift as libc::c_int >
                               31 as libc::c_int {
                            ((*(*sdcard.dma).dma).HISR) &
                                (0x20 as libc::c_int as uint32_t) <<
                                    (*sdcard.dma).flagsShift as libc::c_int -
                                        32 as libc::c_int
                        } else {
                            ((*(*sdcard.dma).dma).LISR) &
                                (0x20 as libc::c_int as uint32_t) <<
                                    (*sdcard.dma).flagsShift as libc::c_int
                        }) != 0 {
                    //Clear both flags after transfer
                    if (*sdcard.dma).flagsShift as libc::c_int >
                           31 as libc::c_int {
                        ::core::ptr::write_volatile(&mut (*(*sdcard.dma).dma).HIFCR
                                                        as *mut uint32_t,
                                                    (0x20 as libc::c_int as
                                                         uint32_t) <<
                                                        (*sdcard.dma).flagsShift
                                                            as libc::c_int -
                                                            32 as libc::c_int)
                    } else {
                        ::core::ptr::write_volatile(&mut (*(*sdcard.dma).dma).LIFCR
                                                        as *mut uint32_t,
                                                    (0x20 as libc::c_int as
                                                         uint32_t) <<
                                                        (*sdcard.dma).flagsShift
                                                            as libc::c_int)
                    }
                    if (*sdcard.dma).flagsShift as libc::c_int >
                           31 as libc::c_int {
                        ::core::ptr::write_volatile(&mut (*(*sdcard.dma).dma).HIFCR
                                                        as *mut uint32_t,
                                                    (0x10 as libc::c_int as
                                                         uint32_t) <<
                                                        (*sdcard.dma).flagsShift
                                                            as libc::c_int -
                                                            32 as libc::c_int)
                    } else {
                        ::core::ptr::write_volatile(&mut (*(*sdcard.dma).dma).LIFCR
                                                        as *mut uint32_t,
                                                    (0x10 as libc::c_int as
                                                         uint32_t) <<
                                                        (*sdcard.dma).flagsShift
                                                            as libc::c_int)
                    }
                    // Drain anything left in the Rx FIFO (we didn't read it during the write)
                    while LL_SPI_IsActiveFlag_RXNE(sdcard.instance) != 0 { }
                    // Wait for the final bit to be transmitted
                    while spiIsBusBusy(sdcard.instance) { }
                    LL_SPI_DisableDMAReq_TX(sdcard.instance);
                    sendComplete = 1 as libc::c_int != 0
                }
                if !sdcard.useDMAForTx {
                    // Send another chunk
                    spiTransfer(sdcard.instance,
                                sdcard.pendingOperation.buffer.offset((256 as
                                                                           libc::c_int
                                                                           *
                                                                           sdcard.pendingOperation.chunkIndex
                                                                               as
                                                                               libc::c_int)
                                                                          as
                                                                          isize),
                                0 as *mut uint8_t, 256 as libc::c_int);
                    sdcard.pendingOperation.chunkIndex =
                        sdcard.pendingOperation.chunkIndex.wrapping_add(1);
                    sendComplete =
                        sdcard.pendingOperation.chunkIndex as libc::c_int ==
                            512 as libc::c_int / 256 as libc::c_int
                }
                if !sendComplete { break ; }
                // Finish up by sending the CRC and checking the SD-card's acceptance/rejectance
                if sdcard_sendDataBlockFinish() {
                    // The SD card is now busy committing that write to the card
                    sdcard.state = SDCARD_STATE_WAITING_FOR_WRITE;
                    sdcard.operationStartTime = millis();
                    // Since we've transmitted the buffer we can go ahead and tell the caller their operation is complete
                    if sdcard.pendingOperation.callback.is_some() {
                        sdcard.pendingOperation.callback.expect("non-null function pointer")(SDCARD_BLOCK_OPERATION_WRITE,
                                                                                             sdcard.pendingOperation.blockIndex,
                                                                                             sdcard.pendingOperation.buffer,
                                                                                             sdcard.pendingOperation.callbackData);
                    }
                    break ;
                } else {
                    /* Our write was rejected! This could be due to a bad address but we hope not to attempt that, so assume
                     * the card is broken and needs reset.
                     */
                    sdcard_reset();
                    // Announce write failure:
                    if sdcard.pendingOperation.callback.is_some() {
                        sdcard.pendingOperation.callback.expect("non-null function pointer")(SDCARD_BLOCK_OPERATION_WRITE,
                                                                                             sdcard.pendingOperation.blockIndex,
                                                                                             0
                                                                                                 as
                                                                                                 *mut uint8_t,
                                                                                             sdcard.pendingOperation.callbackData); // Assume the card is good if it can complete a write
                    }
                }
            }
            7 => {
                if sdcard_waitForIdle(8 as libc::c_int) {
                    sdcard.failureCount = 0 as libc::c_int as uint8_t;
                    // Still more blocks left to write in a multi-block chain?
                    if sdcard.multiWriteBlocksRemain >
                           1 as libc::c_int as libc::c_uint {
                        sdcard.multiWriteBlocksRemain =
                            sdcard.multiWriteBlocksRemain.wrapping_sub(1);
                        sdcard.multiWriteNextBlock =
                            sdcard.multiWriteNextBlock.wrapping_add(1);
                        sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS
                    } else if sdcard.multiWriteBlocksRemain ==
                                  1 as libc::c_int as libc::c_uint {
                        // This function changes the sd card state for us whether immediately succesful or delayed:
                        if sdcard_endWriteBlocks() as libc::c_uint ==
                               SDCARD_OPERATION_SUCCESS as libc::c_int as
                                   libc::c_uint {
                            sdcard_deselect();
                        }
                    } else {
                        sdcard.state = SDCARD_STATE_READY;
                        sdcard_deselect();
                    }
                    break ;
                } else {
                    if !(millis() >
                             sdcard.operationStartTime.wrapping_add(250 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint))
                       {
                        break ;
                    }
                    /*
                 * The caller has already been told that their write has completed, so they will have discarded
                 * their buffer and have no hope of retrying the operation. But this should be very rare and it allows
                 * them to reuse their buffer milliseconds faster than they otherwise would.
                 */
                    sdcard_reset(); // Assume the card is good if it can complete a read
                }
            }
            5 => {
                match sdcard_receiveDataBlock(sdcard.pendingOperation.buffer,
                                              512 as libc::c_int) as
                          libc::c_uint {
                    0 => {
                        sdcard_deselect();
                        sdcard.state = SDCARD_STATE_READY;
                        sdcard.failureCount = 0 as libc::c_int as uint8_t;
                        if sdcard.pendingOperation.callback.is_some() {
                            sdcard.pendingOperation.callback.expect("non-null function pointer")(SDCARD_BLOCK_OPERATION_READ,
                                                                                                 sdcard.pendingOperation.blockIndex,
                                                                                                 sdcard.pendingOperation.buffer,
                                                                                                 sdcard.pendingOperation.callbackData);
                        }
                        break ;
                    }
                    1 => {
                        if millis() <=
                               sdcard.operationStartTime.wrapping_add(100 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
                           {
                            break ;
                        }
                    }
                    2 => { }
                    _ => { break ; }
                }
                // Timeout has expired, so fall through to convert to a fatal error
                sdcard_deselect();
                sdcard_reset();
                if sdcard.pendingOperation.callback.is_some() {
                    sdcard.pendingOperation.callback.expect("non-null function pointer")(SDCARD_BLOCK_OPERATION_READ,
                                                                                         sdcard.pendingOperation.blockIndex,
                                                                                         0
                                                                                             as
                                                                                             *mut uint8_t,
                                                                                         sdcard.pendingOperation.callbackData);
                }
            }
            9 => {
                if sdcard_waitForIdle(8 as libc::c_int) {
                    sdcard_deselect();
                    sdcard.state = SDCARD_STATE_READY;
                    break ;
                } else {
                    if !(millis() >
                             sdcard.operationStartTime.wrapping_add(250 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint))
                       {
                        break ;
                    }
                    sdcard_reset();
                }
            }
            0 | _ => { break ; }
        }
    }
    // Timeout not reached yet so keep waiting
    if sdcard.state as libc::c_uint >=
           SDCARD_STATE_RESET as libc::c_int as libc::c_uint &&
           (sdcard.state as libc::c_uint) <
               SDCARD_STATE_READY as libc::c_int as libc::c_uint &&
           millis().wrapping_sub(sdcard.operationStartTime) >
               200 as libc::c_int as libc::c_uint {
        sdcard_reset();
    }
    return sdcard_isReady();
}
// Is the card's initialization taking too long?
/* *
 * Write the 512-byte block from the given buffer into the block with the given index.
 *
 * If the write does not complete immediately, your callback will be called later. If the write was successful, the
 * buffer pointer will be the same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * Returns:
 *     SDCARD_OPERATION_IN_PROGRESS - Your buffer is currently being transmitted to the card and your callback will be
 *                                    called later to report the completion. The buffer pointer must remain valid until
 *                                    that time.
 *     SDCARD_OPERATION_SUCCESS     - Your buffer has been transmitted to the card now.
 *     SDCARD_OPERATION_BUSY        - The card is already busy and cannot accept your write
 *     SDCARD_OPERATION_FAILURE     - Your write was rejected by the card, card will be reset
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_writeBlock(mut blockIndex: uint32_t,
                                           mut buffer: *mut uint8_t,
                                           mut callback:
                                               sdcard_operationCompleteCallback_c,
                                           mut callbackData: uint32_t)
 -> sdcardOperationStatus_e {
    let mut status: uint8_t = 0;
    loop 
         // Now we've entered the ready state, we can try again
         {
        match sdcard.state as libc::c_uint {
            8 => {
                // Do we need to cancel the previous multi-block write?
                if !(blockIndex != sdcard.multiWriteNextBlock) { break ; }
                if sdcard_endWriteBlocks() as libc::c_uint ==
                       SDCARD_OPERATION_SUCCESS as libc::c_int as libc::c_uint
                   {
                    continue ;
                }
                return SDCARD_OPERATION_BUSY
            }
            4 => {
                // We're not continuing a multi-block write so we need to send a single-block write command
                sdcard_select();
                // Standard size cards use byte addressing, high capacity cards use block addressing
                status =
                    sdcard_sendCommand(24 as libc::c_int as uint8_t,
                                       if sdcard.highCapacity as libc::c_int
                                              != 0 {
                                           blockIndex
                                       } else {
                                           blockIndex.wrapping_mul(512 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint)
                                       }); // (for non-DMA transfers) we've sent chunk #0 already
                if status as libc::c_int != 0 as libc::c_int {
                    sdcard_deselect();
                    sdcard_reset();
                    return SDCARD_OPERATION_FAILURE
                }
                break ;
            }
            _ => { return SDCARD_OPERATION_BUSY }
        }
    }
    sdcard_sendDataBlockBegin(buffer,
                              sdcard.state as libc::c_uint ==
                                  SDCARD_STATE_WRITING_MULTIPLE_BLOCKS as
                                      libc::c_int as libc::c_uint);
    sdcard.pendingOperation.buffer = buffer;
    sdcard.pendingOperation.blockIndex = blockIndex;
    sdcard.pendingOperation.callback = callback;
    sdcard.pendingOperation.callbackData = callbackData;
    sdcard.pendingOperation.chunkIndex = 1 as libc::c_int as uint8_t;
    sdcard.state = SDCARD_STATE_SENDING_WRITE;
    return SDCARD_OPERATION_IN_PROGRESS;
}
/* *
 * Begin writing a series of consecutive blocks beginning at the given block index. This will allow (but not require)
 * the SD card to pre-erase the number of blocks you specifiy, which can allow the writes to complete faster.
 *
 * Afterwards, just call sdcard_writeBlock() as normal to write those blocks consecutively.
 *
 * It's okay to abort the multi-block write at any time by writing to a non-consecutive address, or by performing a read.
 *
 * Returns:
 *     SDCARD_OPERATION_SUCCESS     - Multi-block write has been queued
 *     SDCARD_OPERATION_BUSY        - The card is already busy and cannot accept your write
 *     SDCARD_OPERATION_FAILURE     - A fatal error occured, card will be reset
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_beginWriteBlocks(mut blockIndex: uint32_t,
                                                 mut blockCount: uint32_t)
 -> sdcardOperationStatus_e {
    if sdcard.state as libc::c_uint !=
           SDCARD_STATE_READY as libc::c_int as libc::c_uint {
        if sdcard.state as libc::c_uint ==
               SDCARD_STATE_WRITING_MULTIPLE_BLOCKS as libc::c_int as
                   libc::c_uint {
            if blockIndex == sdcard.multiWriteNextBlock {
                // Assume that the caller wants to continue the multi-block write they already have in progress!
                return SDCARD_OPERATION_SUCCESS
            } else {
                if sdcard_endWriteBlocks() as libc::c_uint !=
                       SDCARD_OPERATION_SUCCESS as libc::c_int as libc::c_uint
                   {
                    return SDCARD_OPERATION_BUSY
                }
            }
            // Else we've completed the previous multi-block write and can fall through to start the new one
        } else { return SDCARD_OPERATION_BUSY }
    }
    sdcard_select();
    if sdcard_sendAppCommand(23 as libc::c_int as uint8_t, blockCount) as
           libc::c_int == 0 as libc::c_int &&
           sdcard_sendCommand(25 as libc::c_int as uint8_t,
                              (if sdcard.highCapacity as libc::c_int != 0 {
                                   blockIndex
                               } else {
                                   blockIndex.wrapping_mul(512 as libc::c_int
                                                               as
                                                               libc::c_uint)
                               })) as libc::c_int == 0 as libc::c_int {
        sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
        sdcard.multiWriteBlocksRemain = blockCount;
        sdcard.multiWriteNextBlock = blockIndex;
        // Leave the card selected
        return SDCARD_OPERATION_SUCCESS
    } else {
        sdcard_deselect();
        sdcard_reset();
        return SDCARD_OPERATION_FAILURE
    };
}
/* *
 * Read the 512-byte block with the given index into the given 512-byte buffer.
 *
 * When the read completes, your callback will be called. If the read was successful, the buffer pointer will be the
 * same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * You must keep the pointer to the buffer valid until the operation completes!
 *
 * Returns:
 *     true - The operation was successfully queued for later completion, your callback will be called later
 *     false - The operation could not be started due to the card being busy (try again later).
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_readBlock(mut blockIndex: uint32_t,
                                          mut buffer: *mut uint8_t,
                                          mut callback:
                                              sdcard_operationCompleteCallback_c,
                                          mut callbackData: uint32_t)
 -> bool {
    if sdcard.state as libc::c_uint !=
           SDCARD_STATE_READY as libc::c_int as libc::c_uint {
        if sdcard.state as libc::c_uint ==
               SDCARD_STATE_WRITING_MULTIPLE_BLOCKS as libc::c_int as
                   libc::c_uint {
            if sdcard_endWriteBlocks() as libc::c_uint !=
                   SDCARD_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
                return 0 as libc::c_int != 0
            }
        } else { return 0 as libc::c_int != 0 }
    }
    sdcard_select();
    // Standard size cards use byte addressing, high capacity cards use block addressing
    let mut status: uint8_t =
        sdcard_sendCommand(17 as libc::c_int as uint8_t,
                           if sdcard.highCapacity as libc::c_int != 0 {
                               blockIndex
                           } else {
                               blockIndex.wrapping_mul(512 as libc::c_int as
                                                           libc::c_uint)
                           });
    if status as libc::c_int == 0 as libc::c_int {
        sdcard.pendingOperation.buffer = buffer;
        sdcard.pendingOperation.blockIndex = blockIndex;
        sdcard.pendingOperation.callback = callback;
        sdcard.pendingOperation.callbackData = callbackData;
        sdcard.state = SDCARD_STATE_READING;
        sdcard.operationStartTime = millis();
        // Leave the card selected for the whole transaction
        return 1 as libc::c_int != 0
    } else { sdcard_deselect(); return 0 as libc::c_int != 0 };
}
/* *
 * Returns true if the SD card has successfully completed its startup procedures.
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_isInitialized() -> bool {
    return sdcard.state as libc::c_uint >=
               SDCARD_STATE_READY as libc::c_int as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn sdcard_getMetadata() -> *const sdcardMetadata_t {
    return &mut sdcard.metadata;
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
