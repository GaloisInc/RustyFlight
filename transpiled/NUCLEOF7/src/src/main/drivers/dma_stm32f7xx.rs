use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_NVIC_EnableIRQ(IRQn: IRQn_Type);
    #[no_mangle]
    fn HAL_NVIC_SetPriority(IRQn: IRQn_Type, PreemptPriority: uint32_t,
                            SubPriority: uint32_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
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
 * DMA descriptors.
 */
// Initialized in run_static_initializers
static mut dmaDescriptors: [dmaChannelDescriptor_t; 16] =
    [dmaChannelDescriptor_t{dma: 0 as *mut DMA_TypeDef,
                            ref_0: 0 as *mut DMA_Stream_TypeDef,
                            stream: 0,
                            irqHandlerCallback: None,
                            flagsShift: 0,
                            irqN: WWDG_IRQn,
                            userParam: 0,
                            owner: OWNER_FREE,
                            resourceIndex: 0,
                            completeFlag: 0,}; 16];
/*
 * DMA IRQ Handlers
 */
#[no_mangle]
pub unsafe extern "C" fn DMA1_Stream0_IRQHandler() {
    let index: uint8_t =
        (DMA1_ST0_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Stream1_IRQHandler() {
    let index: uint8_t =
        (DMA1_ST1_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Stream2_IRQHandler() {
    let index: uint8_t =
        (DMA1_ST2_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Stream3_IRQHandler() {
    let index: uint8_t =
        (DMA1_ST3_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Stream4_IRQHandler() {
    let index: uint8_t =
        (DMA1_ST4_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Stream5_IRQHandler() {
    let index: uint8_t =
        (DMA1_ST5_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Stream6_IRQHandler() {
    let index: uint8_t =
        (DMA1_ST6_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Stream7_IRQHandler() {
    let index: uint8_t =
        (DMA1_ST7_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Stream0_IRQHandler() {
    let index: uint8_t =
        (DMA2_ST0_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Stream1_IRQHandler() {
    let index: uint8_t =
        (DMA2_ST1_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Stream2_IRQHandler() {
    let index: uint8_t =
        (DMA2_ST2_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Stream3_IRQHandler() {
    let index: uint8_t =
        (DMA2_ST3_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Stream4_IRQHandler() {
    let index: uint8_t =
        (DMA2_ST4_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Stream5_IRQHandler() {
    let index: uint8_t =
        (DMA2_ST5_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Stream6_IRQHandler() {
    let index: uint8_t =
        (DMA2_ST6_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Stream7_IRQHandler() {
    let index: uint8_t =
        (DMA2_ST7_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
unsafe extern "C" fn enableDmaClock(mut index: libc::c_int) {
    let rcc: uint32_t =
        if dmaDescriptors[index as usize].dma ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x6000
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut DMA_TypeDef {
            ((0x1 as libc::c_uint)) << 21 as libc::c_uint
        } else { ((0x1 as libc::c_uint)) << 22 as libc::c_uint };
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
                                     as libc::c_uint | rcc) as uint32_t as
                                    uint32_t);
    /* Delay after an RCC peripheral clock enabling */
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR & rcc);
}
#[no_mangle]
pub unsafe extern "C" fn dmaInit(mut identifier: dmaIdentifier_e,
                                 mut owner: resourceOwner_e,
                                 mut resourceIndex: uint8_t) {
    let index: libc::c_int =
        (identifier as
             libc::c_uint).wrapping_sub(1 as libc::c_int as libc::c_uint) as
            libc::c_int;
    enableDmaClock(index);
    dmaDescriptors[index as usize].owner = owner;
    dmaDescriptors[index as usize].resourceIndex = resourceIndex;
}
#[no_mangle]
pub unsafe extern "C" fn dmaSetHandler(mut identifier: dmaIdentifier_e,
                                       mut callback:
                                           dmaCallbackHandlerFuncPtr,
                                       mut priority: uint32_t,
                                       mut userParam: uint32_t) {
    let index: libc::c_int =
        (identifier as
             libc::c_uint).wrapping_sub(1 as libc::c_int as libc::c_uint) as
            libc::c_int;
    enableDmaClock(index);
    dmaDescriptors[index as usize].irqHandlerCallback = callback;
    dmaDescriptors[index as usize].userParam = userParam;
    HAL_NVIC_SetPriority(dmaDescriptors[index as usize].irqN,
                         priority >>
                             (4 as libc::c_int as
                                  libc::c_uint).wrapping_sub((7 as libc::c_int
                                                                  as
                                                                  libc::c_uint).wrapping_sub(0x5
                                                                                                 as
                                                                                                 libc::c_uint))
                             >> 4 as libc::c_int,
                         (priority &
                              (0xf as libc::c_int >>
                                   (7 as libc::c_int as
                                        libc::c_uint).wrapping_sub(0x5 as
                                                                       libc::c_uint))
                                  as libc::c_uint) >> 4 as libc::c_int);
    HAL_NVIC_EnableIRQ(dmaDescriptors[index as usize].irqN);
}
#[no_mangle]
pub unsafe extern "C" fn dmaGetOwner(mut identifier: dmaIdentifier_e)
 -> resourceOwner_e {
    return dmaDescriptors[(identifier as
                               libc::c_uint).wrapping_sub(1 as libc::c_int as
                                                              libc::c_uint) as
                              usize].owner;
}
#[no_mangle]
pub unsafe extern "C" fn dmaGetResourceIndex(mut identifier: dmaIdentifier_e)
 -> uint8_t {
    return dmaDescriptors[(identifier as
                               libc::c_uint).wrapping_sub(1 as libc::c_int as
                                                              libc::c_uint) as
                              usize].resourceIndex;
}
#[no_mangle]
pub unsafe extern "C" fn dmaGetIdentifier(mut stream:
                                              *const DMA_Stream_TypeDef)
 -> dmaIdentifier_e {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < DMA_LAST_HANDLER as libc::c_int {
        if dmaDescriptors[i as usize].ref_0 ==
               stream as *mut DMA_Stream_TypeDef {
            return (i + 1 as libc::c_int) as dmaIdentifier_e
        }
        i += 1
    }
    return DMA_NONE;
}
#[no_mangle]
pub unsafe extern "C" fn dmaGetRefByIdentifier(identifier: dmaIdentifier_e)
 -> *mut DMA_Stream_TypeDef {
    return dmaDescriptors[(identifier as
                               libc::c_uint).wrapping_sub(1 as libc::c_int as
                                                              libc::c_uint) as
                              usize].ref_0;
}
#[no_mangle]
pub unsafe extern "C" fn dmaGetDescriptorByIdentifier(identifier:
                                                          dmaIdentifier_e)
 -> *mut dmaChannelDescriptor_t {
    return &mut *dmaDescriptors.as_mut_ptr().offset((identifier as
                                                         libc::c_uint).wrapping_sub(1
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        libc::c_uint)
                                                        as isize) as
               *mut dmaChannelDescriptor_t;
}
#[no_mangle]
pub unsafe extern "C" fn dmaGetChannel(channel: uint8_t) -> uint32_t {
    return (channel as
                uint32_t).wrapping_mul(2 as libc::c_int as libc::c_uint) <<
               24 as libc::c_int;
}
unsafe extern "C" fn run_static_initializers() {
    dmaDescriptors =
        [{
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x10
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 0 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            0 as libc::c_int as uint8_t,
                                        irqN: DMA1_Stream0_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x28
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 1 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            6 as libc::c_int as uint8_t,
                                        irqN: DMA1_Stream1_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x40
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 2 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            16 as libc::c_int as uint8_t,
                                        irqN: DMA1_Stream2_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x58
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 3 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            22 as libc::c_int as uint8_t,
                                        irqN: DMA1_Stream3_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x70
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 4 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            32 as libc::c_int as uint8_t,
                                        irqN: DMA1_Stream4_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x88
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 5 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            38 as libc::c_int as uint8_t,
                                        irqN: DMA1_Stream5_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0xa0
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 6 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            48 as libc::c_int as uint8_t,
                                        irqN: DMA1_Stream6_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0xb8
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 7 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            54 as libc::c_int as uint8_t,
                                        irqN: DMA1_Stream7_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x10
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 0 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            0 as libc::c_int as uint8_t,
                                        irqN: DMA2_Stream0_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x28
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 1 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            6 as libc::c_int as uint8_t,
                                        irqN: DMA2_Stream1_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x40
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 2 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            16 as libc::c_int as uint8_t,
                                        irqN: DMA2_Stream2_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x58
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 3 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            22 as libc::c_int as uint8_t,
                                        irqN: DMA2_Stream3_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x70
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 4 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            32 as libc::c_int as uint8_t,
                                        irqN: DMA2_Stream4_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x88
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 5 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            38 as libc::c_int as uint8_t,
                                        irqN: DMA2_Stream5_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0xa0
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 6 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            48 as libc::c_int as uint8_t,
                                        irqN: DMA2_Stream6_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x6400
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0xb8
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                as *mut DMA_Stream_TypeDef,
                                        stream: 7 as libc::c_int as uint8_t,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            54 as libc::c_int as uint8_t,
                                        irqN: DMA2_Stream7_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
