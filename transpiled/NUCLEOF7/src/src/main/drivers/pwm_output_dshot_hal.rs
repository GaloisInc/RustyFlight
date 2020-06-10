use ::libc;
extern "C" {
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    fn LL_TIM_StructInit(TIM_InitStruct: *mut LL_TIM_InitTypeDef);
    #[no_mangle]
    fn LL_TIM_Init(TIMx: *mut TIM_TypeDef,
                   TIM_InitStruct: *mut LL_TIM_InitTypeDef) -> ErrorStatus;
    #[no_mangle]
    fn LL_TIM_OC_StructInit(TIM_OC_InitStruct: *mut LL_TIM_OC_InitTypeDef);
    #[no_mangle]
    fn LL_TIM_OC_Init(TIMx: *mut TIM_TypeDef, Channel: uint32_t,
                      TIM_OC_InitStruct: *mut LL_TIM_OC_InitTypeDef)
     -> ErrorStatus;
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
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn timerChCCR(timHw: *const timerHardware_t) -> *mut timCCR_t;
    #[no_mangle]
    fn timerClock(tim: *mut TIM_TypeDef) -> uint32_t;
    // TODO - just for migration
    #[no_mangle]
    fn timerRCC(tim: *mut TIM_TypeDef) -> rccPeriphTag_t;
    #[no_mangle]
    fn timerDmaSource(channel: uint8_t) -> uint16_t;
    #[no_mangle]
    fn timerGetTIMNumber(tim: *const TIM_TypeDef) -> int8_t;
    #[no_mangle]
    fn timerLookupChannelIndex(channel: uint16_t) -> uint8_t;
    #[no_mangle]
    static mut useBurstDshot: bool;
    // function pointer used to encode a digital motor value into the DMA buffer representation
    #[no_mangle]
    fn prepareDshotPacket(motor: *mut motorDmaOutput_t) -> uint16_t;
    #[no_mangle]
    static mut loadDmaBuffer: Option<loadDmaBufferFn>;
    #[no_mangle]
    fn getDshotHz(pwmProtocolType: motorPwmProtocolTypes_e) -> uint32_t;
    #[no_mangle]
    fn pwmDshotCommandIsQueued() -> bool;
    #[no_mangle]
    fn pwmDshotCommandIsProcessing() -> bool;
    #[no_mangle]
    fn pwmGetDshotCommand(index: uint8_t) -> uint8_t;
    #[no_mangle]
    fn pwmDshotCommandOutputIsEnabled(motorCount: uint8_t) -> bool;
    #[no_mangle]
    fn dmaInit(identifier: dmaIdentifier_e, owner: resourceOwner_e,
               resourceIndex: uint8_t);
    #[no_mangle]
    fn dmaSetHandler(identifier: dmaIdentifier_e,
                     callback: dmaCallbackHandlerFuncPtr, priority: uint32_t,
                     userParam: uint32_t);
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type IRQn_Type = libc::c_int;
pub const SPDIF_RX_IRQn: IRQn_Type = 97;
pub const I2C4_ER_IRQn: IRQn_Type = 96;
pub const I2C4_EV_IRQn: IRQn_Type = 95;
pub const CEC_IRQn: IRQn_Type = 94;
pub const LPTIM1_IRQn: IRQn_Type = 93;
pub const QUADSPI_IRQn: IRQn_Type = 92;
pub const SAI2_IRQn: IRQn_Type = 91;
pub const DMA2D_IRQn: IRQn_Type = 90;
pub const LTDC_ER_IRQn: IRQn_Type = 89;
pub const LTDC_IRQn: IRQn_Type = 88;
pub const SAI1_IRQn: IRQn_Type = 87;
pub const SPI6_IRQn: IRQn_Type = 86;
pub const SPI5_IRQn: IRQn_Type = 85;
pub const SPI4_IRQn: IRQn_Type = 84;
pub const UART8_IRQn: IRQn_Type = 83;
pub const UART7_IRQn: IRQn_Type = 82;
pub const FPU_IRQn: IRQn_Type = 81;
pub const RNG_IRQn: IRQn_Type = 80;
pub const DCMI_IRQn: IRQn_Type = 78;
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
pub const CAN2_SCE_IRQn: IRQn_Type = 66;
pub const CAN2_RX1_IRQn: IRQn_Type = 65;
pub const CAN2_RX0_IRQn: IRQn_Type = 64;
pub const CAN2_TX_IRQn: IRQn_Type = 63;
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
  * @brief TIM
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint32_t,
    pub ARR: uint32_t,
    pub RCR: uint32_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint32_t,
    pub DMAR: uint32_t,
    pub OR: uint32_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
}
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
pub type size_t = libc::c_ulong;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_TIM_InitTypeDef {
    pub Prescaler: uint16_t,
    pub CounterMode: uint32_t,
    pub Autoreload: uint32_t,
    pub ClockDivision: uint32_t,
    pub RepetitionCounter: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_TIM_OC_InitTypeDef {
    pub OCMode: uint32_t,
    pub OCState: uint32_t,
    pub OCNState: uint32_t,
    pub CompareValue: uint32_t,
    pub OCPolarity: uint32_t,
    pub OCNPolarity: uint32_t,
    pub OCIdleState: uint32_t,
    pub OCNIdleState: uint32_t,
}
pub type assert_failed_DMA_Stream_TypeDef_has_padding = [libc::c_char; 1];
pub type assert_failed_DMA_TypeDef_has_padding = [libc::c_char; 1];
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
pub type rccPeriphTag_t = uint8_t;
// 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)
pub type timCCR_t = uint32_t;
pub type timerUsageFlag_e = libc::c_uint;
pub const TIM_USE_BEEPER: timerUsageFlag_e = 64;
pub const TIM_USE_TRANSPONDER: timerUsageFlag_e = 32;
pub const TIM_USE_LED: timerUsageFlag_e = 16;
pub const TIM_USE_SERVO: timerUsageFlag_e = 8;
pub const TIM_USE_MOTOR: timerUsageFlag_e = 4;
pub const TIM_USE_PWM: timerUsageFlag_e = 2;
pub const TIM_USE_PPM: timerUsageFlag_e = 1;
pub const TIM_USE_NONE: timerUsageFlag_e = 0;
pub const TIM_USE_ANY: timerUsageFlag_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub alternateFunction: uint8_t,
    pub dmaRef: *mut DMA_Stream_TypeDef,
    pub dmaChannel: uint32_t,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Stream_TypeDef,
    pub dmaTimUPChannel: uint32_t,
    pub dmaTimUPIrqHandler: uint8_t,
}
pub type timerHardware_t = timerHardware_s;
pub type C2RustUnnamed = libc::c_uint;
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed = 2;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed = 1;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed = 0;
pub type motorPwmProtocolTypes_e = libc::c_uint;
pub const PWM_TYPE_MAX: motorPwmProtocolTypes_e = 10;
pub const PWM_TYPE_PROSHOT1000: motorPwmProtocolTypes_e = 9;
pub const PWM_TYPE_DSHOT1200: motorPwmProtocolTypes_e = 8;
pub const PWM_TYPE_DSHOT600: motorPwmProtocolTypes_e = 7;
pub const PWM_TYPE_DSHOT300: motorPwmProtocolTypes_e = 6;
pub const PWM_TYPE_DSHOT150: motorPwmProtocolTypes_e = 5;
pub const PWM_TYPE_BRUSHED: motorPwmProtocolTypes_e = 4;
pub const PWM_TYPE_MULTISHOT: motorPwmProtocolTypes_e = 3;
pub const PWM_TYPE_ONESHOT42: motorPwmProtocolTypes_e = 2;
pub const PWM_TYPE_ONESHOT125: motorPwmProtocolTypes_e = 1;
pub const PWM_TYPE_STANDARD: motorPwmProtocolTypes_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDmaTimer_t {
    pub timer: *mut TIM_TypeDef,
    pub dmaBurstRef: *mut DMA_Stream_TypeDef,
    pub dmaBurstLength: uint16_t,
    pub dmaBurstBuffer: [uint32_t; 72],
    pub timerDmaSources: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDmaOutput_t {
    pub ioTag: ioTag_t,
    pub timerHardware: *const timerHardware_t,
    pub value: uint16_t,
    pub timerDmaSource: uint16_t,
    pub configured: bool,
    pub timer: *mut motorDmaTimer_t,
    pub requestTelemetry: bool,
    pub dmaBuffer: [uint32_t; 18],
}
pub type loadDmaBufferFn
    =
    unsafe extern "C" fn(_: *mut uint32_t, _: libc::c_int, _: uint16_t)
        -> uint8_t;
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
// TIMUP
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
// clear stream address
#[inline]
unsafe extern "C" fn LL_EX_TIM_CC_EnableNChannel(mut TIMx: *mut TIM_TypeDef,
                                                 mut Channel: uint32_t) {
    LL_TIM_CC_EnableChannel(TIMx,
                            (4 as libc::c_int as
                                 libc::c_uint).wrapping_mul(Channel));
}
#[inline]
unsafe extern "C" fn LL_EX_TIM_DisableIT(mut TIMx: *mut TIM_TypeDef,
                                         mut Sources: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).DIER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !Sources) as uint32_t
                                    as uint32_t);
}
#[inline]
unsafe extern "C" fn LL_EX_TIM_EnableIT(mut TIMx: *mut TIM_TypeDef,
                                        mut Sources: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).DIER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Sources) as uint32_t as
                                    uint32_t);
}
#[inline]
unsafe extern "C" fn LL_EX_DMA_SetDataLength(mut DMAx_Streamy:
                                                 *mut DMA_Stream_TypeDef,
                                             mut NbData: uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMAx_Streamy).NDTR as *mut uint32_t,
                                (*DMAx_Streamy).NDTR &
                                    !((0xffff as libc::c_uint) <<
                                          0 as libc::c_uint) | NbData);
}
#[inline]
unsafe extern "C" fn LL_EX_DMA_EnableIT_TC(mut DMAx_Streamy:
                                               *mut DMA_Stream_TypeDef) {
    ::core::ptr::write_volatile(&mut (*DMAx_Streamy).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*DMAx_Streamy).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         4 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
#[inline]
unsafe extern "C" fn LL_EX_DMA_DisableStream(mut DMAx_Streamy:
                                                 *mut DMA_Stream_TypeDef) {
    ::core::ptr::write_volatile(&mut (*DMAx_Streamy).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*DMAx_Streamy).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
#[inline]
unsafe extern "C" fn LL_EX_DMA_EnableStream(mut DMAx_Streamy:
                                                *mut DMA_Stream_TypeDef) {
    ::core::ptr::write_volatile(&mut (*DMAx_Streamy).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*DMAx_Streamy).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
#[inline]
unsafe extern "C" fn LL_EX_DMA_DeInit(mut DMAx_Streamy:
                                          *mut DMA_Stream_TypeDef)
 -> uint32_t {
    let mut DMA: *mut DMA_TypeDef = LL_EX_DMA_Stream_to_DMA(DMAx_Streamy);
    let Stream: uint32_t = LL_EX_DMA_Stream_to_Stream(DMAx_Streamy);
    return LL_DMA_DeInit(DMA, Stream);
}
#[inline]
unsafe extern "C" fn LL_EX_DMA_Init(mut DMAx_Streamy: *mut DMA_Stream_TypeDef,
                                    mut DMA_InitStruct:
                                        *mut LL_DMA_InitTypeDef) -> uint32_t {
    let mut DMA: *mut DMA_TypeDef = LL_EX_DMA_Stream_to_DMA(DMAx_Streamy);
    let Stream: uint32_t = LL_EX_DMA_Stream_to_Stream(DMAx_Streamy);
    return LL_DMA_Init(DMA, Stream, DMA_InitStruct);
}
#[inline]
unsafe extern "C" fn LL_EX_DMA_Stream_to_Stream(mut DMAx_Streamy:
                                                    *mut DMA_Stream_TypeDef)
 -> uint32_t {
    let firstStreamOffset: size_t =
        ::core::mem::size_of::<DMA_TypeDef>() as libc::c_ulong;
    let streamSize: size_t =
        ::core::mem::size_of::<DMA_Stream_TypeDef>() as libc::c_ulong;
    return ((DMAx_Streamy as uint32_t & 0xff as libc::c_uint) as
                libc::c_ulong).wrapping_sub(firstStreamOffset).wrapping_div(streamSize)
               as uint32_t;
}
#[inline]
unsafe extern "C" fn LL_EX_DMA_Stream_to_DMA(mut DMAx_Streamy:
                                                 *mut DMA_Stream_TypeDef)
 -> *mut DMA_TypeDef {
    return (DMAx_Streamy as uint32_t & !(0xff as libc::c_uint)) as
               *mut DMA_TypeDef;
}
static mut OFFSET_TAB_CCMRx: [uint8_t; 9] =
    [0 as libc::c_uint as uint8_t, 0 as libc::c_uint as uint8_t,
     0 as libc::c_uint as uint8_t, 0 as libc::c_uint as uint8_t,
     0x4 as libc::c_uint as uint8_t, 0x4 as libc::c_uint as uint8_t,
     0x4 as libc::c_uint as uint8_t, 0x3c as libc::c_uint as uint8_t,
     0x3c as libc::c_uint as uint8_t];
static mut SHIFT_TAB_OCxx: [uint8_t; 9] =
    [0 as libc::c_uint as uint8_t, 0 as libc::c_uint as uint8_t,
     8 as libc::c_uint as uint8_t, 0 as libc::c_uint as uint8_t,
     0 as libc::c_uint as uint8_t, 0 as libc::c_uint as uint8_t,
     8 as libc::c_uint as uint8_t, 0 as libc::c_uint as uint8_t,
     8 as libc::c_uint as uint8_t];
/* !< Transfer is done to 4 registers starting from the DMA burst base address */
/* !< Transfer is done to 5 registers starting from the DMA burst base address */
/* !< Transfer is done to 6 registers starting from the DMA burst base address */
/* !< Transfer is done to 7 registers starting from the DMA burst base address */
/* !< Transfer is done to 1 registers starting from the DMA burst base address */
/* !< Transfer is done to 9 registers starting from the DMA burst base address */
/* !< Transfer is done to 10 registers starting from the DMA burst base address */
/* !< Transfer is done to 11 registers starting from the DMA burst base address */
/* !< Transfer is done to 12 registers starting from the DMA burst base address */
/* !< Transfer is done to 13 registers starting from the DMA burst base address */
/* !< Transfer is done to 14 registers starting from the DMA burst base address */
/* !< Transfer is done to 15 registers starting from the DMA burst base address */
/* !< Transfer is done to 16 registers starting from the DMA burst base address */
/* !< Transfer is done to 17 registers starting from the DMA burst base address */
/* !< Transfer is done to 18 registers starting from the DMA burst base address */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_TIM2_ITR1_RMP_TIM8  TIM2 Internal Trigger1 Remap TIM8
  * @{
  */
/* !< TIM2_ITR1 is connected to TIM8_TRGO */
/* !< TIM2_ITR1 is connected to ETH_PTP */
/* !< TIM2_ITR1 is connected to OTG_FS SOF */
/* !< TIM2_ITR1 is connected to OTG_HS SOF */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_TIM5_TI4_RMP  TIM5 External Input Ch4 Remap
  * @{
  */
/* !< TIM5 channel 4 is connected to GPIO */
/* !< TIM5 channel 4 is connected to LSI internal clock */
/* !< TIM5 channel 4 is connected to LSE */
/* !< TIM5 channel 4 is connected to RTC wakeup interrupt */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_TIM11_TI1_RMP  TIM11 External Input Capture 1 Remap
  * @{
  */
/* !< TIM11 channel 1 is connected to GPIO */
/* !< TIM11 channel 1 is connected to SPDIFRX */
/* !< TIM11 channel 1 is connected to HSE */
/* !< TIM11 channel 1 is connected to MCO1 */
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup TIM_LL_Exported_Macros TIM Exported Macros
  * @{
  */
/* * @defgroup TIM_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/* *
  * @brief  Write a value in TIM register.
  * @param  __INSTANCE__ TIM Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
/* *
  * @brief  Read a value in TIM register.
  * @param  __INSTANCE__ TIM Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EM_Exported_Macros Exported_Macros
  * @{
  */
/* *
  * @brief  HELPER macro retrieving the UIFCPY flag from the counter value.
  * @note ex: @ref __LL_TIM_GETFLAG_UIFCPY (@ref LL_TIM_GetCounter ());
  * @note  Relevant only if UIF flag remapping has been enabled  (UIF status bit is copied
  *        to TIMx_CNT register bit 31)
  * @param  __CNT__ Counter value
  * @retval UIF status bit
  */
/* *
  * @brief  HELPER macro calculating DTG[0:7] in the TIMx_BDTR register to achieve the requested dead time duration.
  * @note ex: @ref __LL_TIM_CALC_DEADTIME (80000000, @ref LL_TIM_GetClockDivision (), 120);
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __CKD__ This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  * @param  __DT__ deadtime duration (in ns)
  * @retval DTG[0:7]
  */
/* *
  * @brief  HELPER macro calculating the prescaler value to achieve the required counter clock frequency.
  * @note ex: @ref __LL_TIM_CALC_PSC (80000000, 1000000);
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __CNTCLK__ counter clock frequency (in Hz)
  * @retval Prescaler value  (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  HELPER macro calculating the auto-reload value to achieve the required output signal frequency.
  * @note ex: @ref __LL_TIM_CALC_ARR (1000000, @ref LL_TIM_GetPrescaler (), 10000);
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __FREQ__ output signal frequency (in Hz)
  * @retval  Auto-reload value  (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  HELPER macro calculating the compare value required to achieve the required timer output compare active/inactive delay.
  * @note ex: @ref __LL_TIM_CALC_DELAY (1000000, @ref LL_TIM_GetPrescaler (), 10);
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __DELAY__ timer output compare active/inactive delay (in us)
  * @retval Compare value  (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  HELPER macro calculating the auto-reload value to achieve the required pulse duration (when the timer operates in one pulse mode).
  * @note ex: @ref __LL_TIM_CALC_PULSE (1000000, @ref LL_TIM_GetPrescaler (), 10, 20);
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __DELAY__ timer output compare active/inactive delay (in us)
  * @param  __PULSE__ pulse duration (in us)
  * @retval Auto-reload value  (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  HELPER macro retrieving the ratio of the input capture prescaler
  * @note ex: @ref __LL_TIM_GET_ICPSC_RATIO (@ref LL_TIM_IC_GetPrescaler ());
  * @param  __ICPSC__ This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ICPSC_DIV1
  *         @arg @ref LL_TIM_ICPSC_DIV2
  *         @arg @ref LL_TIM_ICPSC_DIV4
  *         @arg @ref LL_TIM_ICPSC_DIV8
  * @retval Input capture prescaler ratio (1, 2, 4 or 8)
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup TIM_LL_Exported_Functions TIM Exported Functions
  * @{
  */
/* * @defgroup TIM_LL_EF_Time_Base Time Base configuration
  * @{
  */
/* *
  * @brief  Enable timer counter.
  * @rmtoll CR1          CEN           LL_TIM_EnableCounter
  * @param  TIMx Timer instance
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_EnableCounter(mut TIMx: *mut TIM_TypeDef) {
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Disable timer counter.
  * @rmtoll CR1          CEN           LL_TIM_DisableCounter
  * @param  TIMx Timer instance
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_DisableCounter(mut TIMx: *mut TIM_TypeDef) {
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Enable auto-reload (ARR) preload.
  * @rmtoll CR1          ARPE          LL_TIM_EnableARRPreload
  * @param  TIMx Timer instance
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_EnableARRPreload(mut TIMx: *mut TIM_TypeDef) {
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Set the counter value.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @rmtoll CNT          CNT           LL_TIM_SetCounter
  * @param  TIMx Timer instance
  * @param  Counter Counter value (between Min_Data=0 and Max_Data=0xFFFF or 0xFFFFFFFF)
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_SetCounter(mut TIMx: *mut TIM_TypeDef,
                                       mut Counter: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).CNT as *mut uint32_t, Counter);
}
/* *
  * @brief  Enable capture/compare channels.
  * @rmtoll CCER         CC1E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC1NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC2E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC2NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC3E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC3NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC4E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC5E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC6E          LL_TIM_CC_EnableChannel
  * @param  TIMx Timer instance
  * @param  Channels This parameter can be a combination of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_CC_EnableChannel(mut TIMx: *mut TIM_TypeDef,
                                             mut Channels: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Channels) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Disable fast mode for the output channel.
  * @rmtoll CCMR1        OC1FE          LL_TIM_OC_DisableFast\n
  *         CCMR1        OC2FE          LL_TIM_OC_DisableFast\n
  *         CCMR2        OC3FE          LL_TIM_OC_DisableFast\n
  *         CCMR2        OC4FE          LL_TIM_OC_DisableFast\n
  *         CCMR3        OC5FE          LL_TIM_OC_DisableFast\n
  *         CCMR3        OC6FE          LL_TIM_OC_DisableFast
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_OC_DisableFast(mut TIMx: *mut TIM_TypeDef,
                                           mut Channel: uint32_t) {
    let mut iChannel: uint8_t =
        if Channel == (0x1 as libc::c_uint) << 0 as libc::c_uint {
            0 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 2 as libc::c_uint {
            1 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 4 as libc::c_uint {
            2 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 6 as libc::c_uint {
            3 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 8 as libc::c_uint {
            4 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 10 as libc::c_uint {
            5 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 12 as libc::c_uint {
            6 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 16 as libc::c_uint {
            7 as libc::c_uint
        } else { 8 as libc::c_uint } as uint8_t;
    let mut pReg: *mut uint32_t =
        (&mut (*TIMx).CCMR1 as *mut uint32_t as
             uint32_t).wrapping_add(OFFSET_TAB_CCMRx[iChannel as usize] as
                                        libc::c_uint) as *mut uint32_t;
    *pReg &=
        !(((0x1 as libc::c_uint) << 2 as libc::c_uint) <<
              SHIFT_TAB_OCxx[iChannel as usize] as libc::c_int);
}
/* *
  * @brief  Enable compare register (TIMx_CCRx) preload for the output channel.
  * @rmtoll CCMR1        OC1PE          LL_TIM_OC_EnablePreload\n
  *         CCMR1        OC2PE          LL_TIM_OC_EnablePreload\n
  *         CCMR2        OC3PE          LL_TIM_OC_EnablePreload\n
  *         CCMR2        OC4PE          LL_TIM_OC_EnablePreload\n
  *         CCMR3        OC5PE          LL_TIM_OC_EnablePreload\n
  *         CCMR3        OC6PE          LL_TIM_OC_EnablePreload
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_OC_EnablePreload(mut TIMx: *mut TIM_TypeDef,
                                             mut Channel: uint32_t) {
    let mut iChannel: uint8_t =
        if Channel == (0x1 as libc::c_uint) << 0 as libc::c_uint {
            0 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 2 as libc::c_uint {
            1 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 4 as libc::c_uint {
            2 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 6 as libc::c_uint {
            3 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 8 as libc::c_uint {
            4 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 10 as libc::c_uint {
            5 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 12 as libc::c_uint {
            6 as libc::c_uint
        } else if Channel == (0x1 as libc::c_uint) << 16 as libc::c_uint {
            7 as libc::c_uint
        } else { 8 as libc::c_uint } as uint8_t;
    let mut pReg: *mut uint32_t =
        (&mut (*TIMx).CCMR1 as *mut uint32_t as
             uint32_t).wrapping_add(OFFSET_TAB_CCMRx[iChannel as usize] as
                                        libc::c_uint) as *mut uint32_t;
    *pReg |=
        ((0x1 as libc::c_uint) << 3 as libc::c_uint) <<
            SHIFT_TAB_OCxx[iChannel as usize] as libc::c_int;
}
/* *
  * @brief  Enable the outputs (set the MOE bit in TIMx_BDTR register).
  * @note The MOE bit in TIMx_BDTR register allows to enable /disable the outputs by
  *       software and is reset in case of break or break2 event
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @rmtoll BDTR         MOE           LL_TIM_EnableAllOutputs
  * @param  TIMx Timer instance
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_EnableAllOutputs(mut TIMx: *mut TIM_TypeDef) {
    ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* TIM_BREAK_INPUT_SUPPORT */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_DMA_Burst_Mode DMA burst mode configuration
  * @{
  */
/* *
  * @brief  Configures the timer DMA burst feature.
  * @note Macro @ref IS_TIM_DMABURST_INSTANCE(TIMx) can be used to check whether or
  *       not a timer instance supports the DMA burst mode.
  * @rmtoll DCR          DBL           LL_TIM_ConfigDMABurst\n
  *         DCR          DBA           LL_TIM_ConfigDMABurst
  * @param  TIMx Timer instance
  * @param  DMABurstBaseAddress This parameter can be one of the following values:
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_SMCR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_DIER
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_SR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_EGR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCER
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CNT
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_PSC
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_ARR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_RCR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR3
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR4
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_BDTR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR3
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR5
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR6
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_OR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_AF1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_AF2
  * @param  DMABurstLength This parameter can be one of the following values:
  *         @arg @ref LL_TIM_DMABURST_LENGTH_1TRANSFER
  *         @arg @ref LL_TIM_DMABURST_LENGTH_2TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_3TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_4TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_5TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_6TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_7TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_8TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_9TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_10TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_11TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_12TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_13TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_14TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_15TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_16TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_17TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_18TRANSFERS
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_ConfigDMABurst(mut TIMx: *mut TIM_TypeDef,
                                           mut DMABurstBaseAddress: uint32_t,
                                           mut DMABurstLength: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).DCR as *mut uint32_t,
                                (*TIMx).DCR &
                                    !((0x1f as libc::c_uint) <<
                                          8 as libc::c_uint |
                                          (0x1f as libc::c_uint) <<
                                              0 as libc::c_uint) |
                                    (DMABurstBaseAddress | DMABurstLength));
}
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_DMA_Management DMA-Management
  * @{
  */
/* *
  * @brief  Enable update DMA request (UDE).
  * @rmtoll DIER         UDE           LL_TIM_EnableDMAReq_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_EnableDMAReq_UPDATE(mut TIMx: *mut TIM_TypeDef) {
    ::core::ptr::write_volatile(&mut (*TIMx).DIER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Disable update DMA request (UDE).
  * @rmtoll DIER         UDE           LL_TIM_DisableDMAReq_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_TIM_DisableDMAReq_UPDATE(mut TIMx: *mut TIM_TypeDef) {
    ::core::ptr::write_volatile(&mut (*TIMx).DIER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).DIER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t);
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
#[link_section = ".fastram_bss"]
static mut dmaMotorTimerCount: uint8_t = 0 as libc::c_int as uint8_t;
#[link_section = ".fastram_bss"]
static mut dmaMotorTimers: [motorDmaTimer_t; 8] =
    [motorDmaTimer_t{timer: 0 as *const TIM_TypeDef as *mut TIM_TypeDef,
                     dmaBurstRef:
                         0 as *const DMA_Stream_TypeDef as
                             *mut DMA_Stream_TypeDef,
                     dmaBurstLength: 0,
                     dmaBurstBuffer: [0; 72],
                     timerDmaSources: 0,}; 8];
#[link_section = ".fastram_bss"]
static mut dmaMotors: [motorDmaOutput_t; 8] =
    [motorDmaOutput_t{ioTag: 0,
                      timerHardware: 0 as *const timerHardware_t,
                      value: 0,
                      timerDmaSource: 0,
                      configured: false,
                      timer:
                          0 as *const motorDmaTimer_t as *mut motorDmaTimer_t,
                      requestTelemetry: false,
                      dmaBuffer: [0; 18],}; 8];
#[no_mangle]
pub unsafe extern "C" fn getMotorDmaOutput(mut index: uint8_t)
 -> *mut motorDmaOutput_t {
    return &mut *dmaMotors.as_mut_ptr().offset(index as isize) as
               *mut motorDmaOutput_t;
}
#[no_mangle]
pub unsafe extern "C" fn getTimerIndex(mut timer: *mut TIM_TypeDef)
 -> uint8_t {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < dmaMotorTimerCount as libc::c_int {
        if dmaMotorTimers[i as usize].timer == timer { return i as uint8_t }
        i += 1
    }
    let fresh0 = dmaMotorTimerCount;
    dmaMotorTimerCount = dmaMotorTimerCount.wrapping_add(1);
    dmaMotorTimers[fresh0 as usize].timer = timer;
    return (dmaMotorTimerCount as libc::c_int - 1 as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn pwmWriteDshotInt(mut index: uint8_t,
                                          mut value: uint16_t) {
    let motor: *mut motorDmaOutput_t =
        &mut *dmaMotors.as_mut_ptr().offset(index as isize) as
            *mut motorDmaOutput_t;
    if !(*motor).configured { return }
    /*If there is a command ready to go overwrite the value and send that instead*/
    if pwmDshotCommandIsProcessing() {
        value = pwmGetDshotCommand(index) as uint16_t;
        if value != 0 {
            ::core::ptr::write_volatile(&mut (*motor).requestTelemetry as
                                            *mut bool, 1 as libc::c_int != 0)
        }
    }
    (*motor).value = value;
    let mut packet: uint16_t = prepareDshotPacket(motor);
    let mut bufferSize: uint8_t = 0;
    if useBurstDshot {
        bufferSize =
            loadDmaBuffer.expect("non-null function pointer")(&mut *(*(*motor).timer).dmaBurstBuffer.as_mut_ptr().offset((timerLookupChannelIndex
                                                                                                                              as
                                                                                                                              unsafe extern "C" fn(_:
                                                                                                                                                       uint16_t)
                                                                                                                                  ->
                                                                                                                                      uint8_t)((*(*motor).timerHardware).channel
                                                                                                                                                   as
                                                                                                                                                   uint16_t)
                                                                                                                             as
                                                                                                                             isize),
                                                              4 as
                                                                  libc::c_int,
                                                              packet);
        (*(*motor).timer).dmaBurstLength =
            (bufferSize as libc::c_int * 4 as libc::c_int) as uint16_t
    } else {
        bufferSize =
            loadDmaBuffer.expect("non-null function pointer")((*motor).dmaBuffer.as_mut_ptr(),
                                                              1 as
                                                                  libc::c_int,
                                                              packet);
        (*(*motor).timer).timerDmaSources =
            ((*(*motor).timer).timerDmaSources as libc::c_int |
                 (*motor).timerDmaSource as libc::c_int) as uint16_t;
        LL_EX_DMA_SetDataLength((*(*motor).timerHardware).dmaRef,
                                bufferSize as uint32_t);
        LL_EX_DMA_EnableStream((*(*motor).timerHardware).dmaRef);
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmCompleteDshotMotorUpdate(mut motorCount:
                                                         uint8_t) {
    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if pwmDshotCommandIsQueued() {
        if !pwmDshotCommandOutputIsEnabled(motorCount) { return }
    }
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < dmaMotorTimerCount as libc::c_int {
        if useBurstDshot {
            LL_EX_DMA_SetDataLength(dmaMotorTimers[i as usize].dmaBurstRef,
                                    dmaMotorTimers[i as usize].dmaBurstLength
                                        as uint32_t);
            LL_EX_DMA_EnableStream(dmaMotorTimers[i as usize].dmaBurstRef);
            /* configure the DMA Burst Mode */
            LL_TIM_ConfigDMABurst(dmaMotorTimers[i as usize].timer,
                                  (0x8 as libc::c_uint) << 0 as libc::c_uint |
                                      (0x4 as libc::c_uint) <<
                                          0 as libc::c_uint |
                                      (0x1 as libc::c_uint) <<
                                          0 as libc::c_uint,
                                  (0x2 as libc::c_uint) << 8 as libc::c_uint |
                                      (0x1 as libc::c_uint) <<
                                          8 as libc::c_uint);
            /* Enable the TIM DMA Request */
            LL_TIM_EnableDMAReq_UPDATE(dmaMotorTimers[i as usize].timer);
        } else {
            /* Reset timer counter */
            LL_TIM_SetCounter(dmaMotorTimers[i as usize].timer,
                              0 as libc::c_int as uint32_t);
            /* Enable channel DMA requests */
            LL_EX_TIM_EnableIT(dmaMotorTimers[i as usize].timer,
                               dmaMotorTimers[i as usize].timerDmaSources as
                                   uint32_t);
            dmaMotorTimers[i as usize].timerDmaSources =
                0 as libc::c_int as uint16_t
        }
        i += 1
    };
}
unsafe extern "C" fn motor_DMA_IRQHandler(mut descriptor:
                                              *mut dmaChannelDescriptor_t) {
    if if (*descriptor).flagsShift as libc::c_int > 31 as libc::c_int {
           ((*(*descriptor).dma).HISR) &
               (0x20 as libc::c_int as uint32_t) <<
                   (*descriptor).flagsShift as libc::c_int - 32 as libc::c_int
       } else {
           ((*(*descriptor).dma).LISR) &
               (0x20 as libc::c_int as uint32_t) <<
                   (*descriptor).flagsShift as libc::c_int
       } != 0 {
        let motor: *mut motorDmaOutput_t =
            &mut *dmaMotors.as_mut_ptr().offset((*descriptor).userParam as
                                                    isize) as
                *mut motorDmaOutput_t;
        if useBurstDshot {
            LL_EX_DMA_DisableStream((*(*motor).timerHardware).dmaTimUPRef);
            LL_TIM_DisableDMAReq_UPDATE((*(*motor).timerHardware).tim);
        } else {
            LL_EX_DMA_DisableStream((*(*motor).timerHardware).dmaRef);
            LL_EX_TIM_DisableIT((*(*motor).timerHardware).tim,
                                (*motor).timerDmaSource as uint32_t);
        }
        if (*descriptor).flagsShift as libc::c_int > 31 as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*descriptor).dma).HIFCR as
                                            *mut uint32_t,
                                        (0x20 as libc::c_int as uint32_t) <<
                                            (*descriptor).flagsShift as
                                                libc::c_int -
                                                32 as libc::c_int)
        } else {
            ::core::ptr::write_volatile(&mut (*(*descriptor).dma).LIFCR as
                                            *mut uint32_t,
                                        (0x20 as libc::c_int as uint32_t) <<
                                            (*descriptor).flagsShift as
                                                libc::c_int)
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmDshotMotorHardwareConfig(mut timerHardware:
                                                         *const timerHardware_t,
                                                     mut motorIndex: uint8_t,
                                                     mut pwmProtocolType:
                                                         motorPwmProtocolTypes_e,
                                                     mut output: uint8_t) {
    let mut dmaRef: *mut DMA_Stream_TypeDef = 0 as *mut DMA_Stream_TypeDef;
    if useBurstDshot {
        dmaRef = (*timerHardware).dmaTimUPRef
    } else { dmaRef = (*timerHardware).dmaRef }
    if dmaRef.is_null() { return }
    let mut oc_init: LL_TIM_OC_InitTypeDef =
        LL_TIM_OC_InitTypeDef{OCMode: 0,
                              OCState: 0,
                              OCNState: 0,
                              CompareValue: 0,
                              OCPolarity: 0,
                              OCNPolarity: 0,
                              OCIdleState: 0,
                              OCNIdleState: 0,};
    let mut dma_init: LL_DMA_InitTypeDef =
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
    let motor: *mut motorDmaOutput_t =
        &mut *dmaMotors.as_mut_ptr().offset(motorIndex as isize) as
            *mut motorDmaOutput_t;
    (*motor).timerHardware = timerHardware;
    let mut timer: *mut TIM_TypeDef = (*timerHardware).tim;
    let motorIO: IO_t = IOGetByTag((*timerHardware).tag);
    let timerIndex: uint8_t = getTimerIndex(timer);
    let configureTimer: bool =
        timerIndex as libc::c_int ==
            dmaMotorTimerCount as libc::c_int - 1 as libc::c_int;
    IOConfigGPIOAF(motorIO,
                   (0x2 as libc::c_uint |
                        (0x3 as libc::c_uint) << 2 as libc::c_int |
                        (0x2 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, (*timerHardware).alternateFunction);
    if configureTimer {
        let mut init: LL_TIM_InitTypeDef =
            LL_TIM_InitTypeDef{Prescaler: 0,
                               CounterMode: 0,
                               Autoreload: 0,
                               ClockDivision: 0,
                               RepetitionCounter: 0,};
        LL_TIM_StructInit(&mut init);
        RCC_ClockCmd(timerRCC(timer), ENABLE);
        LL_TIM_DisableCounter(timer);
        init.Prescaler =
            (lrintf(timerClock(timer) as libc::c_float /
                        getDshotHz(pwmProtocolType) as libc::c_float +
                        0.01f32) - 1 as libc::c_int as libc::c_long) as
                uint16_t;
        init.Autoreload =
            if pwmProtocolType as libc::c_uint ==
                   PWM_TYPE_PROSHOT1000 as libc::c_int as libc::c_uint {
                96 as libc::c_int
            } else { 19 as libc::c_int } as uint32_t;
        init.ClockDivision = 0 as libc::c_uint;
        init.RepetitionCounter = 0 as libc::c_int as uint8_t;
        init.CounterMode = 0 as libc::c_uint;
        LL_TIM_Init(timer, &mut init);
    }
    LL_TIM_OC_StructInit(&mut oc_init);
    oc_init.OCMode =
        (0x4 as libc::c_uint) << 4 as libc::c_uint |
            (0x2 as libc::c_uint) << 4 as libc::c_uint;
    if output as libc::c_int & TIMER_OUTPUT_N_CHANNEL as libc::c_int != 0 {
        oc_init.OCNState = (0x1 as libc::c_uint) << 0 as libc::c_uint;
        oc_init.OCNIdleState = 0 as libc::c_uint;
        oc_init.OCNPolarity =
            if output as libc::c_int & TIMER_OUTPUT_INVERTED as libc::c_int !=
                   0 {
                ((0x1 as libc::c_uint)) << 1 as libc::c_uint
            } else { 0 as libc::c_uint }
    } else {
        oc_init.OCState = (0x1 as libc::c_uint) << 0 as libc::c_uint;
        oc_init.OCIdleState = (0x1 as libc::c_uint) << 8 as libc::c_uint;
        oc_init.OCPolarity =
            if output as libc::c_int & TIMER_OUTPUT_INVERTED as libc::c_int !=
                   0 {
                ((0x1 as libc::c_uint)) << 1 as libc::c_uint
            } else { 0 as libc::c_uint }
    }
    oc_init.CompareValue = 0 as libc::c_int as uint32_t;
    let mut channel: uint32_t = 0 as libc::c_int as uint32_t;
    match (*timerHardware).channel as libc::c_int {
        0 => { channel = (0x1 as libc::c_uint) << 0 as libc::c_uint }
        4 => { channel = (0x1 as libc::c_uint) << 4 as libc::c_uint }
        8 => { channel = (0x1 as libc::c_uint) << 8 as libc::c_uint }
        12 => { channel = (0x1 as libc::c_uint) << 12 as libc::c_uint }
        _ => { }
    }
    LL_TIM_OC_Init(timer, channel, &mut oc_init);
    LL_TIM_OC_EnablePreload(timer, channel);
    LL_TIM_OC_DisableFast(timer, channel);
    if output as libc::c_int & TIMER_OUTPUT_N_CHANNEL as libc::c_int != 0 {
        LL_EX_TIM_CC_EnableNChannel(timer, channel);
    } else { LL_TIM_CC_EnableChannel(timer, channel); }
    if configureTimer {
        LL_TIM_EnableAllOutputs(timer);
        LL_TIM_EnableARRPreload(timer);
        LL_TIM_EnableCounter(timer);
    }
    (*motor).timer =
        &mut *dmaMotorTimers.as_mut_ptr().offset(timerIndex as isize) as
            *mut motorDmaTimer_t;
    if useBurstDshot {
        (*(*motor).timer).dmaBurstRef = dmaRef;
        if !configureTimer {
            (*motor).configured = 1 as libc::c_int != 0;
            return
        }
    } else {
        (*motor).timerDmaSource = timerDmaSource((*timerHardware).channel);
        (*(*motor).timer).timerDmaSources =
            ((*(*motor).timer).timerDmaSources as libc::c_int &
                 !((*motor).timerDmaSource as libc::c_int)) as uint16_t
    }
    LL_EX_DMA_DeInit(dmaRef);
    LL_DMA_StructInit(&mut dma_init);
    if useBurstDshot {
        dmaInit((*timerHardware).dmaTimUPIrqHandler as dmaIdentifier_e,
                OWNER_TIMUP,
                timerGetTIMNumber((*timerHardware).tim) as uint8_t);
        dmaSetHandler((*timerHardware).dmaTimUPIrqHandler as dmaIdentifier_e,
                      Some(motor_DMA_IRQHandler as
                               unsafe extern "C" fn(_:
                                                        *mut dmaChannelDescriptor_t)
                                   -> ()),
                      (((1 as libc::c_int) <<
                            (4 as libc::c_int as
                                 libc::c_uint).wrapping_sub((7 as libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_sub(0x5
                                                                                                as
                                                                                                libc::c_uint))
                            |
                            2 as libc::c_int &
                                0xf as libc::c_int >>
                                    (7 as libc::c_int as
                                         libc::c_uint).wrapping_sub(0x5 as
                                                                        libc::c_uint))
                           << 4 as libc::c_int & 0xf0 as libc::c_int) as
                          uint32_t, motorIndex as uint32_t);
        dma_init.Channel = (*timerHardware).dmaTimUPChannel;
        dma_init.MemoryOrM2MDstAddress =
            (*(*motor).timer).dmaBurstBuffer.as_mut_ptr() as uint32_t;
        dma_init.FIFOThreshold = (0x3 as libc::c_uint) << 0 as libc::c_uint;
        dma_init.PeriphOrM2MSrcAddress =
            &mut (*(*timerHardware).tim).DMAR as *mut uint32_t as uint32_t
    } else {
        dmaInit((*timerHardware).dmaIrqHandler as dmaIdentifier_e,
                OWNER_MOTOR,
                (motorIndex as libc::c_int + 1 as libc::c_int) as uint8_t);
        dmaSetHandler((*timerHardware).dmaIrqHandler as dmaIdentifier_e,
                      Some(motor_DMA_IRQHandler as
                               unsafe extern "C" fn(_:
                                                        *mut dmaChannelDescriptor_t)
                                   -> ()),
                      (((1 as libc::c_int) <<
                            (4 as libc::c_int as
                                 libc::c_uint).wrapping_sub((7 as libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_sub(0x5
                                                                                                as
                                                                                                libc::c_uint))
                            |
                            2 as libc::c_int &
                                0xf as libc::c_int >>
                                    (7 as libc::c_int as
                                         libc::c_uint).wrapping_sub(0x5 as
                                                                        libc::c_uint))
                           << 4 as libc::c_int & 0xf0 as libc::c_int) as
                          uint32_t, motorIndex as uint32_t);
        dma_init.Channel = (*timerHardware).dmaChannel;
        dma_init.MemoryOrM2MDstAddress =
            (*motor).dmaBuffer.as_mut_ptr() as uint32_t;
        dma_init.FIFOThreshold = 0 as libc::c_uint;
        dma_init.PeriphOrM2MSrcAddress = timerChCCR(timerHardware) as uint32_t
    }
    dma_init.Direction = (0x1 as libc::c_uint) << 6 as libc::c_uint;
    dma_init.FIFOMode = (0x1 as libc::c_uint) << 2 as libc::c_uint;
    dma_init.MemBurst = 0 as libc::c_uint;
    dma_init.PeriphBurst = 0 as libc::c_uint;
    dma_init.NbData =
        if pwmProtocolType as libc::c_uint ==
               PWM_TYPE_PROSHOT1000 as libc::c_int as libc::c_uint {
            6 as libc::c_int
        } else { 18 as libc::c_int } as uint32_t;
    dma_init.PeriphOrM2MSrcIncMode = 0 as libc::c_uint;
    dma_init.MemoryOrM2MDstIncMode =
        (0x1 as libc::c_uint) << 10 as libc::c_uint;
    dma_init.PeriphOrM2MSrcDataSize =
        (0x2 as libc::c_uint) << 11 as libc::c_uint;
    dma_init.MemoryOrM2MDstDataSize =
        (0x2 as libc::c_uint) << 13 as libc::c_uint;
    dma_init.Mode = 0 as libc::c_uint;
    dma_init.Priority = (0x2 as libc::c_uint) << 16 as libc::c_uint;
    LL_EX_DMA_Init(dmaRef, &mut dma_init);
    LL_EX_DMA_EnableIT_TC(dmaRef);
    (*motor).configured = 1 as libc::c_int != 0;
}
