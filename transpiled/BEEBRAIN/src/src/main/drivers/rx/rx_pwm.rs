use ::libc;
extern "C" {
    #[no_mangle]
    fn TIM_ICInit(TIMx: *mut TIM_TypeDef,
                  TIM_ICInitStruct: *mut TIM_ICInitTypeDef);
    #[no_mangle]
    fn TIM_ICStructInit(TIM_ICInitStruct: *mut TIM_ICInitTypeDef);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn pwmGetMotors() -> *mut pwmOutputPort_t;
    #[no_mangle]
    fn timerChCCHandlerInit(self_0: *mut timerCCHandlerRec_t,
                            fn_0: Option<timerCCHandlerCallback>);
    #[no_mangle]
    fn timerChOvrHandlerInit(self_0: *mut timerOvrHandlerRec_t,
                             fn_0: Option<timerOvrHandlerCallback>);
    #[no_mangle]
    fn timerChConfigCallbacks(channel: *const timerHardware_t,
                              edgeCallback: *mut timerCCHandlerRec_t,
                              overflowCallback: *mut timerOvrHandlerRec_t);
    #[no_mangle]
    fn timerConfigure(timHw: *const timerHardware_t, period: uint16_t,
                      hz: uint32_t);
    #[no_mangle]
    fn timerClock(tim: *mut TIM_TypeDef) -> uint32_t;
    #[no_mangle]
    fn timerGetByTag(ioTag: ioTag_t) -> *const timerHardware_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
/* * 
  * @brief TIM
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SMCR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DIER: uint16_t,
    pub RESERVED3: uint16_t,
    pub SR: uint16_t,
    pub RESERVED4: uint16_t,
    pub EGR: uint16_t,
    pub RESERVED5: uint16_t,
    pub CCMR1: uint16_t,
    pub RESERVED6: uint16_t,
    pub CCMR2: uint16_t,
    pub RESERVED7: uint16_t,
    pub CCER: uint16_t,
    pub RESERVED8: uint16_t,
    pub CNT: uint16_t,
    pub RESERVED9: uint16_t,
    pub PSC: uint16_t,
    pub RESERVED10: uint16_t,
    pub ARR: uint16_t,
    pub RESERVED11: uint16_t,
    pub RCR: uint16_t,
    pub RESERVED12: uint16_t,
    pub CCR1: uint16_t,
    pub RESERVED13: uint16_t,
    pub CCR2: uint16_t,
    pub RESERVED14: uint16_t,
    pub CCR3: uint16_t,
    pub RESERVED15: uint16_t,
    pub CCR4: uint16_t,
    pub RESERVED16: uint16_t,
    pub BDTR: uint16_t,
    pub RESERVED17: uint16_t,
    pub DCR: uint16_t,
    pub RESERVED18: uint16_t,
    pub DMAR: uint16_t,
    pub RESERVED19: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f10x_gpio.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library.
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
/* * @addtogroup GPIO
  * @{
  */
/* * @defgroup GPIO_Exported_Types
  * @{
  */
/* * 
  * @brief  Output Maximum frequency selection  
  */
pub type C2RustUnnamed = libc::c_uint;
pub const GPIO_Speed_50MHz: C2RustUnnamed = 3;
pub const GPIO_Speed_2MHz: C2RustUnnamed = 2;
pub const GPIO_Speed_10MHz: C2RustUnnamed = 1;
/* * 
  * @brief  Configuration Mode enumeration  
  */
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_Mode_AF_PP: C2RustUnnamed_0 = 24;
pub const GPIO_Mode_AF_OD: C2RustUnnamed_0 = 28;
pub const GPIO_Mode_Out_PP: C2RustUnnamed_0 = 16;
pub const GPIO_Mode_Out_OD: C2RustUnnamed_0 = 20;
pub const GPIO_Mode_IPU: C2RustUnnamed_0 = 72;
pub const GPIO_Mode_IPD: C2RustUnnamed_0 = 40;
pub const GPIO_Mode_IN_FLOATING: C2RustUnnamed_0 = 4;
pub const GPIO_Mode_AIN: C2RustUnnamed_0 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_ICInitTypeDef {
    pub TIM_Channel: uint16_t,
    pub TIM_ICPolarity: uint16_t,
    pub TIM_ICSelection: uint16_t,
    pub TIM_ICPrescaler: uint16_t,
    pub TIM_ICFilter: uint16_t,
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
pub type ioTag_t = uint8_t;
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
pub type captureCompare_t = uint16_t;
pub type timCCR_t = uint16_t;
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
pub struct timerCCHandlerRec_s {
    pub fn_0: Option<timerCCHandlerCallback>,
}
pub type timerCCHandlerCallback
    =
    unsafe extern "C" fn(_: *mut timerCCHandlerRec_s, _: uint16_t) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerOvrHandlerRec_s {
    pub fn_0: Option<timerOvrHandlerCallback>,
    pub next: *mut timerOvrHandlerRec_s,
}
pub type timerOvrHandlerCallback
    =
    unsafe extern "C" fn(_: *mut timerOvrHandlerRec_s, _: uint16_t) -> ();
pub type timerCCHandlerRec_t = timerCCHandlerRec_s;
pub type timerOvrHandlerRec_t = timerOvrHandlerRec_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub dmaRef: *mut DMA_Channel_TypeDef,
    pub dmaIrqHandler: uint8_t,
}
pub type timerHardware_t = timerHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerChannel_t {
    pub ccr: *mut timCCR_t,
    pub tim: *mut TIM_TypeDef,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pwmOutputPort_t {
    pub channel: timerChannel_t,
    pub pulseScale: libc::c_float,
    pub pulseOffset: libc::c_float,
    pub forceOverflow: bool,
    pub enabled: bool,
    pub io: IO_t,
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
pub type inputFilteringMode_e = libc::c_uint;
pub const INPUT_FILTERING_ENABLED: inputFilteringMode_e = 1;
pub const INPUT_FILTERING_DISABLED: inputFilteringMode_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ppmConfig_s {
    pub ioTag: ioTag_t,
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
pub type ppmConfig_t = ppmConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pwmInputPort_t {
    pub mode: pwmInputMode_e,
    pub channel: uint8_t,
    pub state: uint8_t,
    pub missedEvents: uint8_t,
    pub rise: captureCompare_t,
    pub fall: captureCompare_t,
    pub capture: captureCompare_t,
    pub timerHardware: *const timerHardware_t,
    pub edgeCb: timerCCHandlerRec_t,
    pub overflowCb: timerOvrHandlerRec_t,
}
pub type pwmInputMode_e = libc::c_uint;
pub const INPUT_MODE_PWM: pwmInputMode_e = 1;
pub const INPUT_MODE_PPM: pwmInputMode_e = 0;
pub type ppmDevice_t = ppmDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ppmDevice_s {
    pub currentCapture: uint32_t,
    pub currentTime: uint32_t,
    pub deltaTime: uint32_t,
    pub captures: [uint32_t; 12],
    pub largeCounter: uint32_t,
    pub pulseIndex: uint8_t,
    pub numChannels: int8_t,
    pub numChannelsPrevFrame: int8_t,
    pub stableFramesSeenCount: uint8_t,
    pub tracking: bool,
    pub overflowed: bool,
}
pub type eventSource_e = libc::c_uint;
pub const SOURCE_EDGE: eventSource_e = 1;
pub const SOURCE_OVERFLOW: eventSource_e = 0;
pub type ppmISREvent_t = ppmISREvent_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ppmISREvent_s {
    pub capture: uint32_t,
    pub source: eventSource_e,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pwmConfig_s {
    pub ioTags: [ioTag_t; 8],
    pub inputFilteringMode: inputFilteringMode_e,
}
pub type pwmConfig_t = pwmConfig_s;
static mut inputFilteringMode: inputFilteringMode_e =
    INPUT_FILTERING_DISABLED;
static mut pwmInputPorts: [pwmInputPort_t; 8] =
    [pwmInputPort_t{mode: INPUT_MODE_PPM,
                    channel: 0,
                    state: 0,
                    missedEvents: 0,
                    rise: 0,
                    fall: 0,
                    capture: 0,
                    timerHardware: 0 as *const timerHardware_t,
                    edgeCb: timerCCHandlerRec_t{fn_0: None,},
                    overflowCb:
                        timerOvrHandlerRec_t{fn_0: None,
                                             next:
                                                 0 as
                                                     *const timerOvrHandlerRec_s
                                                     as
                                                     *mut timerOvrHandlerRec_s,},};
        8];
static mut captures: [uint16_t; 12] = [0; 12];
static mut ppmFrameCount: uint8_t = 0 as libc::c_int as uint8_t;
static mut lastPPMFrameCount: uint8_t = 0 as libc::c_int as uint8_t;
static mut ppmCountDivisor: uint8_t = 1 as libc::c_int as uint8_t;
static mut ppmDev: ppmDevice_t =
    ppmDevice_t{currentCapture: 0,
                currentTime: 0,
                deltaTime: 0,
                captures: [0; 12],
                largeCounter: 0,
                pulseIndex: 0,
                numChannels: 0,
                numChannelsPrevFrame: 0,
                stableFramesSeenCount: 0,
                tracking: false,
                overflowed: false,};
#[no_mangle]
pub unsafe extern "C" fn isPPMDataBeingReceived() -> bool {
    return ppmFrameCount as libc::c_int != lastPPMFrameCount as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn resetPPMDataReceivedState() {
    lastPPMFrameCount = ppmFrameCount;
}
static mut ppmEvents: [ppmISREvent_t; 20] =
    [ppmISREvent_t{capture: 0, source: SOURCE_OVERFLOW,}; 20];
static mut ppmEventIndex: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn ppmISREvent(mut source: eventSource_e,
                                     mut capture: uint32_t) {
    ppmEventIndex =
        ((ppmEventIndex as libc::c_int + 1 as libc::c_int) as
             libc::c_ulong).wrapping_rem((::core::mem::size_of::<[ppmISREvent_t; 20]>()
                                              as
                                              libc::c_ulong).wrapping_div(::core::mem::size_of::<ppmISREvent_t>()
                                                                              as
                                                                              libc::c_ulong))
            as uint8_t;
    ppmEvents[ppmEventIndex as usize].source = source;
    ppmEvents[ppmEventIndex as usize].capture = capture;
}
unsafe extern "C" fn ppmResetDevice() {
    ppmDev.pulseIndex = 0 as libc::c_int as uint8_t;
    ppmDev.currentCapture = 0 as libc::c_int as uint32_t;
    ppmDev.currentTime = 0 as libc::c_int as uint32_t;
    ppmDev.deltaTime = 0 as libc::c_int as uint32_t;
    ppmDev.largeCounter = 0 as libc::c_int as uint32_t;
    ppmDev.numChannels = -(1 as libc::c_int) as int8_t;
    ppmDev.numChannelsPrevFrame = -(1 as libc::c_int) as int8_t;
    ppmDev.stableFramesSeenCount = 0 as libc::c_int as uint8_t;
    ppmDev.tracking = 0 as libc::c_int != 0;
    ppmDev.overflowed = 0 as libc::c_int != 0;
}
unsafe extern "C" fn ppmOverflowCallback(mut cbRec: *mut timerOvrHandlerRec_t,
                                         mut capture: captureCompare_t) {
    ppmISREvent(SOURCE_OVERFLOW, capture as uint32_t);
    ppmDev.largeCounter =
        (ppmDev.largeCounter as
             libc::c_uint).wrapping_add((capture as libc::c_int +
                                             1 as libc::c_int) as
                                            libc::c_uint) as uint32_t as
            uint32_t;
    if capture as libc::c_int == 0x10000 as libc::c_int - 1 as libc::c_int {
        ppmDev.overflowed = 1 as libc::c_int != 0
    };
}
unsafe extern "C" fn ppmEdgeCallback(mut cbRec: *mut timerCCHandlerRec_t,
                                     mut capture: captureCompare_t) {
    ppmISREvent(SOURCE_EDGE, capture as uint32_t);
    let mut i: int32_t = 0;
    let mut previousTime: uint32_t = ppmDev.currentTime;
    let mut previousCapture: uint32_t = ppmDev.currentCapture;
    //uint32_t previousTime;
    /* Grab the new count */
    let mut currentTime: uint32_t = capture as uint32_t;
    /* Convert to 32-bit timer result */
    currentTime =
        (currentTime as libc::c_uint).wrapping_add(ppmDev.largeCounter) as
            uint32_t as uint32_t;
    if (capture as libc::c_uint) < previousCapture {
        if ppmDev.overflowed {
            currentTime =
                (currentTime as
                     libc::c_uint).wrapping_add(0x10000 as libc::c_int as
                                                    libc::c_uint) as uint32_t
                    as uint32_t
        }
    }
    // Divide value if Oneshot, Multishot or brushed motors are active and the timer is shared
    currentTime = currentTime.wrapping_div(ppmCountDivisor as libc::c_uint);
    /* Capture computation */
    if currentTime > previousTime {
        ppmDev.deltaTime =
            currentTime.wrapping_sub(previousTime.wrapping_add((if ppmDev.overflowed
                                                                       as
                                                                       libc::c_int
                                                                       != 0 {
                                                                    (0x10000
                                                                         as
                                                                         libc::c_int)
                                                                        /
                                                                        ppmCountDivisor
                                                                            as
                                                                            libc::c_int
                                                                } else {
                                                                    0 as
                                                                        libc::c_int
                                                                }) as
                                                                   libc::c_uint))
    } else {
        ppmDev.deltaTime =
            ((0x10000 as libc::c_int / ppmCountDivisor as libc::c_int) as
                 libc::c_uint).wrapping_add(currentTime).wrapping_sub(previousTime)
    }
    ppmDev.overflowed = 0 as libc::c_int != 0;
    /* Store the current measurement */
    ppmDev.currentTime = currentTime;
    ppmDev.currentCapture = capture as uint32_t;
    /* Sync pulse detection */
    if ppmDev.deltaTime > 2700 as libc::c_int as libc::c_uint {
        if ppmDev.pulseIndex as libc::c_int ==
               ppmDev.numChannelsPrevFrame as libc::c_int &&
               ppmDev.pulseIndex as libc::c_int >= 4 as libc::c_int &&
               ppmDev.pulseIndex as libc::c_int <= 12 as libc::c_int {
            /* If we see n simultaneous frames of the same
               number of channels we save it as our frame size */
            if (ppmDev.stableFramesSeenCount as libc::c_int) <
                   25 as libc::c_int {
                ppmDev.stableFramesSeenCount =
                    ppmDev.stableFramesSeenCount.wrapping_add(1)
            } else { ppmDev.numChannels = ppmDev.pulseIndex as int8_t }
        } else { ppmDev.stableFramesSeenCount = 0 as libc::c_int as uint8_t }
        /* We rely on the supervisor to set captureValue to invalid
           if no valid frame is found otherwise we ride over it */
        if ppmDev.pulseIndex as libc::c_int ==
               ppmDev.numChannels as libc::c_int &&
               ppmDev.tracking as libc::c_int != 0 {
            /* Check if the last frame was well formed */
            /* The last frame was well formed */
            i = 0 as libc::c_int;
            while i < ppmDev.numChannels as libc::c_int {
                captures[i as usize] =
                    ppmDev.captures[i as usize] as uint16_t;
                i += 1
            }
            i = ppmDev.numChannels as int32_t;
            while i < 12 as libc::c_int {
                captures[i as usize] = 0 as libc::c_int as uint16_t;
                i += 1
            }
            ppmFrameCount = ppmFrameCount.wrapping_add(1)
        }
        ppmDev.tracking = 1 as libc::c_int != 0;
        ppmDev.numChannelsPrevFrame = ppmDev.pulseIndex as int8_t;
        ppmDev.pulseIndex = 0 as libc::c_int as uint8_t
    } else if ppmDev.tracking {
        /* Valid pulse duration 0.75 to 2.5 ms*/
        if ppmDev.deltaTime > 750 as libc::c_int as libc::c_uint &&
               ppmDev.deltaTime < 2250 as libc::c_int as libc::c_uint &&
               (ppmDev.pulseIndex as libc::c_int) < 12 as libc::c_int {
            ppmDev.captures[ppmDev.pulseIndex as usize] = ppmDev.deltaTime;
            ppmDev.pulseIndex = ppmDev.pulseIndex.wrapping_add(1)
        } else {
            /* Not a valid pulse duration */
            ppmDev.tracking = 0 as libc::c_int != 0;
            i = 0 as libc::c_int;
            while i < 12 as libc::c_int {
                ppmDev.captures[i as usize] = 0 as libc::c_int as uint32_t;
                i += 1
            }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn isPWMDataBeingReceived() -> bool {
    let mut channel: libc::c_int = 0;
    channel = 0 as libc::c_int;
    while channel < 12 as libc::c_int {
        if captures[channel as usize] as libc::c_int != 0 as libc::c_int {
            return 1 as libc::c_int != 0
        }
        channel += 1
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn pwmOverflowCallback(mut cbRec: *mut timerOvrHandlerRec_t,
                                         mut capture: captureCompare_t) {
    let mut pwmInputPort: *mut pwmInputPort_t =
        ({
             let mut __mptr: *const timerOvrHandlerRec_t = cbRec;
             (__mptr as
                  *mut libc::c_char).offset(-(32 as libc::c_ulong as isize))
                 as *mut pwmInputPort_t
         });
    (*pwmInputPort).missedEvents =
        (*pwmInputPort).missedEvents.wrapping_add(1);
    if (*pwmInputPort).missedEvents as libc::c_int > 10 as libc::c_int {
        captures[(*pwmInputPort).channel as usize] =
            0 as libc::c_int as uint16_t;
        (*pwmInputPort).missedEvents = 0 as libc::c_int as uint8_t
    };
}
unsafe extern "C" fn pwmEdgeCallback(mut cbRec: *mut timerCCHandlerRec_t,
                                     mut capture: captureCompare_t) {
    let mut pwmInputPort: *mut pwmInputPort_t =
        ({
             let mut __mptr: *const timerCCHandlerRec_t = cbRec;
             (__mptr as
                  *mut libc::c_char).offset(-(24 as libc::c_ulong as isize))
                 as *mut pwmInputPort_t
         });
    let mut timerHardwarePtr: *const timerHardware_t =
        (*pwmInputPort).timerHardware;
    if (*pwmInputPort).state as libc::c_int == 0 as libc::c_int {
        (*pwmInputPort).rise = capture;
        (*pwmInputPort).state = 1 as libc::c_int as uint8_t;
        pwmICConfig((*timerHardwarePtr).tim, (*timerHardwarePtr).channel,
                    0x2 as libc::c_int as uint16_t);
    } else {
        (*pwmInputPort).fall = capture;
        // compute and store capture
        (*pwmInputPort).capture =
            ((*pwmInputPort).fall as libc::c_int -
                 (*pwmInputPort).rise as libc::c_int) as captureCompare_t;
        captures[(*pwmInputPort).channel as usize] = (*pwmInputPort).capture;
        // switch state
        (*pwmInputPort).state = 0 as libc::c_int as uint8_t;
        pwmICConfig((*timerHardwarePtr).tim, (*timerHardwarePtr).channel,
                    0 as libc::c_int as uint16_t);
        (*pwmInputPort).missedEvents = 0 as libc::c_int as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmICConfig(mut tim: *mut TIM_TypeDef,
                                     mut channel: uint8_t,
                                     mut polarity: uint16_t) {
    let mut TIM_ICInitStructure: TIM_ICInitTypeDef =
        TIM_ICInitTypeDef{TIM_Channel: 0,
                          TIM_ICPolarity: 0,
                          TIM_ICSelection: 0,
                          TIM_ICPrescaler: 0,
                          TIM_ICFilter: 0,};
    TIM_ICStructInit(&mut TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel as uint16_t;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = 0x1 as libc::c_int as uint16_t;
    TIM_ICInitStructure.TIM_ICPrescaler = 0 as libc::c_int as uint16_t;
    if inputFilteringMode as libc::c_uint ==
           INPUT_FILTERING_ENABLED as libc::c_int as libc::c_uint {
        TIM_ICInitStructure.TIM_ICFilter = 0x3 as libc::c_int as uint16_t
    } else { TIM_ICInitStructure.TIM_ICFilter = 0 as libc::c_int as uint16_t }
    TIM_ICInit(tim, &mut TIM_ICInitStructure);
}
#[no_mangle]
pub unsafe extern "C" fn pwmRxInit(mut pwmConfig: *const pwmConfig_t) {
    inputFilteringMode = (*pwmConfig).inputFilteringMode;
    let mut channel: libc::c_int = 0 as libc::c_int;
    while channel < 8 as libc::c_int {
        let mut port: *mut pwmInputPort_t =
            &mut *pwmInputPorts.as_mut_ptr().offset(channel as isize) as
                *mut pwmInputPort_t;
        let mut timer: *const timerHardware_t =
            timerGetByTag((*pwmConfig).ioTags[channel as usize]);
        if !timer.is_null() {
            (*port).state = 0 as libc::c_int as uint8_t;
            (*port).missedEvents = 0 as libc::c_int as uint8_t;
            (*port).channel = channel as uint8_t;
            (*port).mode = INPUT_MODE_PWM;
            (*port).timerHardware = timer;
            let mut io: IO_t =
                IOGetByTag((*pwmConfig).ioTags[channel as usize]);
            IOInit(io, OWNER_PWMINPUT,
                   (channel + 1 as libc::c_int) as uint8_t);
            IOConfigGPIO(io,
                         (GPIO_Mode_IPD as libc::c_int |
                              GPIO_Speed_2MHz as libc::c_int) as ioConfig_t);
            timerConfigure(timer, 0x10000 as libc::c_int as uint16_t,
                           (1 as libc::c_int * 1000000 as libc::c_int) as
                               uint32_t);
            timerChCCHandlerInit(&mut (*port).edgeCb,
                                 Some(pwmEdgeCallback as
                                          unsafe extern "C" fn(_:
                                                                   *mut timerCCHandlerRec_t,
                                                               _:
                                                                   captureCompare_t)
                                              -> ()));
            timerChOvrHandlerInit(&mut (*port).overflowCb,
                                  Some(pwmOverflowCallback as
                                           unsafe extern "C" fn(_:
                                                                    *mut timerOvrHandlerRec_t,
                                                                _:
                                                                    captureCompare_t)
                                               -> ()));
            timerChConfigCallbacks(timer, &mut (*port).edgeCb,
                                   &mut (*port).overflowCb);
            pwmICConfig((*timer).tim, (*timer).channel,
                        0 as libc::c_int as uint16_t);
        }
        /* TODO: maybe fail here if not enough channels? */
        channel += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn ppmAvoidPWMTimerClash(mut pwmTimer:
                                                   *mut TIM_TypeDef) {
    let mut motors: *mut pwmOutputPort_t = pwmGetMotors();
    let mut motorIndex: libc::c_int = 0 as libc::c_int;
    while motorIndex < 8 as libc::c_int {
        if !(*motors.offset(motorIndex as isize)).enabled ||
               (*motors.offset(motorIndex as isize)).channel.tim != pwmTimer {
            motorIndex += 1
        } else {
            ppmCountDivisor =
                timerClock(pwmTimer).wrapping_div(((*pwmTimer).PSC as
                                                       libc::c_int +
                                                       1 as libc::c_int) as
                                                      libc::c_uint) as
                    uint8_t;
            return
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn ppmRxInit(mut ppmConfig: *const ppmConfig_t) {
    ppmResetDevice();
    let mut port: *mut pwmInputPort_t =
        &mut *pwmInputPorts.as_mut_ptr().offset(0 as libc::c_int as isize) as
            *mut pwmInputPort_t;
    let mut timer: *const timerHardware_t = timerGetByTag((*ppmConfig).ioTag);
    if timer.is_null() {
        /* TODO: fail here? */
        return
    }
    ppmAvoidPWMTimerClash((*timer).tim);
    (*port).mode = INPUT_MODE_PPM;
    (*port).timerHardware = timer;
    let mut io: IO_t = IOGetByTag((*ppmConfig).ioTag);
    IOInit(io, OWNER_PPMINPUT, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(io,
                 (GPIO_Mode_IPD as libc::c_int |
                      GPIO_Speed_2MHz as libc::c_int) as ioConfig_t);
    timerConfigure(timer, 0x10000 as libc::c_int as uint16_t,
                   (1 as libc::c_int * 1000000 as libc::c_int) as uint32_t);
    timerChCCHandlerInit(&mut (*port).edgeCb,
                         Some(ppmEdgeCallback as
                                  unsafe extern "C" fn(_:
                                                           *mut timerCCHandlerRec_t,
                                                       _: captureCompare_t)
                                      -> ()));
    timerChOvrHandlerInit(&mut (*port).overflowCb,
                          Some(ppmOverflowCallback as
                                   unsafe extern "C" fn(_:
                                                            *mut timerOvrHandlerRec_t,
                                                        _: captureCompare_t)
                                       -> ()));
    timerChConfigCallbacks(timer, &mut (*port).edgeCb,
                           &mut (*port).overflowCb);
    pwmICConfig((*timer).tim, (*timer).channel, 0 as libc::c_int as uint16_t);
}
#[no_mangle]
pub unsafe extern "C" fn ppmRead(mut channel: uint8_t) -> uint16_t {
    return captures[channel as usize];
}
#[no_mangle]
pub unsafe extern "C" fn pwmRead(mut channel: uint8_t) -> uint16_t {
    return captures[channel as usize];
}
