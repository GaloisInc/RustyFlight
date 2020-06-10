use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    /* *
  * @}
  */
    /* * @defgroup I2C_Exported_Constants
  * @{
  */
    /* * @defgroup I2C_mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_duty_cycle_in_fast_mode 
  * @{
  */
    /* !< I2C fast mode Tlow/Thigh = 16/9 */
    /* !< I2C fast mode Tlow/Thigh = 2 */
    /* *
  * @}
  */
    /* * @defgroup I2C_acknowledgement
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_transfer_direction 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_acknowledged_address 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_registers 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_SMBus_alert_pin_level 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_PEC_position 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_NCAK_position 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_interrupts_definition 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_interrupts_definition 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_flags_definition 
  * @{
  */
    /* * 
  * @brief  SR2 register flags  
  */
    /* * 
  * @brief  SR1 register flags  
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_Events 
  * @{
  */
    /*========================================
     
                     I2C Master Events (Events grouped in order of communication)
                                                        ==========================================*/
/* * 
  * @brief  Communication start
  * 
  * After sending the START condition (I2C_GenerateSTART() function) the master 
  * has to wait for this event. It means that the Start condition has been correctly 
  * released on the I2C bus (the bus is free, no other devices is communicating).
  * 
  */
/* --EV5 */
    /* BUSY, MSL and SB flag */
    /* * 
  * @brief  Address Acknowledge
  * 
  * After checking on EV5 (start condition correctly released on the bus), the 
  * master sends the address of the slave(s) with which it will communicate 
  * (I2C_Send7bitAddress() function, it also determines the direction of the communication: 
  * Master transmitter or Receiver). Then the master has to wait that a slave acknowledges 
  * his address. If an acknowledge is sent on the bus, one of the following events will 
  * be set:
  * 
  *  1) In case of Master Receiver (7-bit addressing): the I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED 
  *     event is set.
  *  
  *  2) In case of Master Transmitter (7-bit addressing): the I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED 
  *     is set
  *  
  *  3) In case of 10-Bit addressing mode, the master (just after generating the START 
  *  and checking on EV5) has to send the header of 10-bit addressing mode (I2C_SendData() 
  *  function). Then master should wait on EV9. It means that the 10-bit addressing 
  *  header has been correctly sent on the bus. Then master should send the second part of 
  *  the 10-bit address (LSB) using the function I2C_Send7bitAddress(). Then master 
  *  should wait for event EV6. 
  *     
  */
    /* --EV6 */
    /* BUSY, MSL, ADDR, TXE and TRA flags */
    /* BUSY, MSL and ADDR flags */
    /* --EV9 */
    /* BUSY, MSL and ADD10 flags */
    /* * 
  * @brief Communication events
  * 
  * If a communication is established (START condition generated and slave address 
  * acknowledged) then the master has to check on one of the following events for 
  * communication procedures:
  *  
  * 1) Master Receiver mode: The master has to wait on the event EV7 then to read 
  *    the data received from the slave (I2C_ReceiveData() function).
  * 
  * 2) Master Transmitter mode: The master has to send data (I2C_SendData() 
  *    function) then to wait on event EV8 or EV8_2.
  *    These two events are similar: 
  *     - EV8 means that the data has been written in the data register and is 
  *       being shifted out.
  *     - EV8_2 means that the data has been physically shifted out and output 
  *       on the bus.
  *     In most cases, using EV8 is sufficient for the application.
  *     Using EV8_2 leads to a slower communication but ensure more reliable test.
  *     EV8_2 is also more suitable than EV8 for testing on the last data transmission 
  *     (before Stop condition generation).
  *     
  *  @note In case the  user software does not guarantee that this event EV7 is 
  *  managed before the current byte end of transfer, then user may check on EV7 
  *  and BTF flag at the same time (ie. (I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_FLAG_BTF)).
  *  In this case the communication may be slower.
  * 
  */
    /* Master RECEIVER mode -----------------------------*/ 
/* --EV7 */
    /* BUSY, MSL and RXNE flags */
    /* Master TRANSMITTER mode --------------------------*/
/* --EV8 */
    /* TRA, BUSY, MSL, TXE flags */
    /* --EV8_2 */
    /* TRA, BUSY, MSL, TXE and BTF flags */
    /*========================================
     
                     I2C Slave Events (Events grouped in order of communication)
                                                        ==========================================*/
    /* * 
  * @brief  Communication start events
  * 
  * Wait on one of these events at the start of the communication. It means that 
  * the I2C peripheral detected a Start condition on the bus (generated by master 
  * device) followed by the peripheral address. The peripheral generates an ACK 
  * condition on the bus (if the acknowledge feature is enabled through function 
  * I2C_AcknowledgeConfig()) and the events listed above are set :
  *  
  * 1) In normal case (only one address managed by the slave), when the address 
  *   sent by the master matches the own address of the peripheral (configured by 
  *   I2C_OwnAddress1 field) the I2C_EVENT_SLAVE_XXX_ADDRESS_MATCHED event is set 
  *   (where XXX could be TRANSMITTER or RECEIVER).
  *    
  * 2) In case the address sent by the master matches the second address of the 
  *   peripheral (configured by the function I2C_OwnAddress2Config() and enabled 
  *   by the function I2C_DualAddressCmd()) the events I2C_EVENT_SLAVE_XXX_SECONDADDRESS_MATCHED 
  *   (where XXX could be TRANSMITTER or RECEIVER) are set.
  *   
  * 3) In case the address sent by the master is General Call (address 0x00) and 
  *   if the General Call is enabled for the peripheral (using function I2C_GeneralCallCmd()) 
  *   the following event is set I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED.   
  * 
  */
    /* --EV1  (all the events below are variants of EV1) */   
/* 1) Case of One Single Address managed by the slave */
    /* BUSY and ADDR flags */
    /* TRA, BUSY, TXE and ADDR flags */
    /* 2) Case of Dual address managed by the slave */
    /* DUALF and BUSY flags */
    /* DUALF, TRA, BUSY and TXE flags */
    /* 3) Case of General Call enabled for the slave */
    /* GENCALL and BUSY flags */
    /* * 
  * @brief  Communication events
  * 
  * Wait on one of these events when EV1 has already been checked and: 
  * 
  * - Slave RECEIVER mode:
  *     - EV2: When the application is expecting a data byte to be received. 
  *     - EV4: When the application is expecting the end of the communication: master 
  *       sends a stop condition and data transmission is stopped.
  *    
  * - Slave Transmitter mode:
  *    - EV3: When a byte has been transmitted by the slave and the application is expecting 
  *      the end of the byte transmission. The two events I2C_EVENT_SLAVE_BYTE_TRANSMITTED and
  *      I2C_EVENT_SLAVE_BYTE_TRANSMITTING are similar. The second one can optionally be 
  *      used when the user software doesn't guarantee the EV3 is managed before the
  *      current byte end of transfer.
  *    - EV3_2: When the master sends a NACK in order to tell slave that data transmission 
  *      shall end (before sending the STOP condition). In this case slave has to stop sending 
  *      data bytes and expect a Stop condition on the bus.
  *      
  *  @note In case the  user software does not guarantee that the event EV2 is 
  *  managed before the current byte end of transfer, then user may check on EV2 
  *  and BTF flag at the same time (ie. (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_BTF)).
  * In this case the communication may be slower.
  *
  */
    /* Slave RECEIVER mode --------------------------*/ 
/* --EV2 */
    /* BUSY and RXNE flags */
    /* --EV4  */
    /* STOPF flag */
    /* Slave TRANSMITTER mode -----------------------*/
/* --EV3 */
    /* TRA, BUSY, TXE and BTF flags */
    /* TRA, BUSY and TXE flags */
    /* --EV3_2 */
    /* AF flag */
    /*===========================      End of Events Description           ==========================================*/
    /* *
  * @}
  */
    /* * @defgroup I2C_own_address1 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_clock_speed 
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_Exported_Macros
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_Exported_Functions
  * @{
  */
    #[no_mangle]
    fn I2C_DeInit(I2Cx: *mut I2C_TypeDef);
    #[no_mangle]
    fn I2C_Init(I2Cx: *mut I2C_TypeDef, I2C_InitStruct: *mut I2C_InitTypeDef);
    #[no_mangle]
    fn I2C_StructInit(I2C_InitStruct: *mut I2C_InitTypeDef);
    #[no_mangle]
    fn I2C_Cmd(I2Cx: *mut I2C_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn I2C_GenerateSTART(I2Cx: *mut I2C_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn I2C_GenerateSTOP(I2Cx: *mut I2C_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn I2C_AcknowledgeConfig(I2Cx: *mut I2C_TypeDef,
                             NewState: FunctionalState);
    #[no_mangle]
    fn I2C_ITConfig(I2Cx: *mut I2C_TypeDef, I2C_IT: uint16_t,
                    NewState: FunctionalState);
    #[no_mangle]
    fn I2C_Send7bitAddress(I2Cx: *mut I2C_TypeDef, Address: uint8_t,
                           I2C_Direction: uint8_t);
    #[no_mangle]
    fn I2C_StretchClockCmd(I2Cx: *mut I2C_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn NVIC_Init(NVIC_InitStruct: *mut NVIC_InitTypeDef);
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
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
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
    fn delayMicroseconds(us: timeUs_t);
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
pub type IRQn = libc::c_int;
pub const USBWakeUp_IRQn: IRQn = 42;
pub const RTCAlarm_IRQn: IRQn = 41;
pub const EXTI15_10_IRQn: IRQn = 40;
pub const USART3_IRQn: IRQn = 39;
pub const USART2_IRQn: IRQn = 38;
pub const USART1_IRQn: IRQn = 37;
pub const SPI2_IRQn: IRQn = 36;
pub const SPI1_IRQn: IRQn = 35;
pub const I2C2_ER_IRQn: IRQn = 34;
pub const I2C2_EV_IRQn: IRQn = 33;
pub const I2C1_ER_IRQn: IRQn = 32;
pub const I2C1_EV_IRQn: IRQn = 31;
pub const TIM4_IRQn: IRQn = 30;
pub const TIM3_IRQn: IRQn = 29;
pub const TIM2_IRQn: IRQn = 28;
pub const TIM1_CC_IRQn: IRQn = 27;
pub const TIM1_TRG_COM_IRQn: IRQn = 26;
pub const TIM1_UP_IRQn: IRQn = 25;
pub const TIM1_BRK_IRQn: IRQn = 24;
pub const EXTI9_5_IRQn: IRQn = 23;
pub const CAN1_SCE_IRQn: IRQn = 22;
pub const CAN1_RX1_IRQn: IRQn = 21;
pub const USB_LP_CAN1_RX0_IRQn: IRQn = 20;
pub const USB_HP_CAN1_TX_IRQn: IRQn = 19;
pub const ADC1_2_IRQn: IRQn = 18;
pub const DMA1_Channel7_IRQn: IRQn = 17;
pub const DMA1_Channel6_IRQn: IRQn = 16;
pub const DMA1_Channel5_IRQn: IRQn = 15;
pub const DMA1_Channel4_IRQn: IRQn = 14;
pub const DMA1_Channel3_IRQn: IRQn = 13;
pub const DMA1_Channel2_IRQn: IRQn = 12;
pub const DMA1_Channel1_IRQn: IRQn = 11;
pub const EXTI4_IRQn: IRQn = 10;
pub const EXTI3_IRQn: IRQn = 9;
pub const EXTI2_IRQn: IRQn = 8;
pub const EXTI1_IRQn: IRQn = 7;
pub const EXTI0_IRQn: IRQn = 6;
pub const RCC_IRQn: IRQn = 5;
pub const FLASH_IRQn: IRQn = 4;
pub const RTC_IRQn: IRQn = 3;
pub const TAMPER_IRQn: IRQn = 2;
pub const PVD_IRQn: IRQn = 1;
pub const WWDG_IRQn: IRQn = 0;
pub const SysTick_IRQn: IRQn = -1;
pub const PendSV_IRQn: IRQn = -2;
pub const DebugMonitor_IRQn: IRQn = -4;
pub const SVCall_IRQn: IRQn = -5;
pub const UsageFault_IRQn: IRQn = -10;
pub const BusFault_IRQn: IRQn = -11;
pub const MemoryManagement_IRQn: IRQn = -12;
pub const NonMaskableInt_IRQn: IRQn = -14;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub OAR1: uint16_t,
    pub RESERVED2: uint16_t,
    pub OAR2: uint16_t,
    pub RESERVED3: uint16_t,
    pub DR: uint16_t,
    pub RESERVED4: uint16_t,
    pub SR1: uint16_t,
    pub RESERVED5: uint16_t,
    pub SR2: uint16_t,
    pub RESERVED6: uint16_t,
    pub CCR: uint16_t,
    pub RESERVED7: uint16_t,
    pub TRISE: uint16_t,
    pub RESERVED8: uint16_t,
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
/* *
  ******************************************************************************
  * @file    stm32f10x_i2c.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the I2C firmware 
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
/* * @addtogroup I2C
  * @{
  */
/* * @defgroup I2C_Exported_Types
  * @{
  */
/* * 
  * @brief  I2C Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_InitTypeDef {
    pub I2C_ClockSpeed: uint32_t,
    pub I2C_Mode: uint16_t,
    pub I2C_DutyCycle: uint16_t,
    pub I2C_OwnAddress1: uint16_t,
    pub I2C_Ack: uint16_t,
    pub I2C_AcknowledgedAddress: uint16_t,
}
/* *
  ******************************************************************************
  * @file    misc.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the miscellaneous
  *          firmware library functions (add-on to CMSIS functions).
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
/* * @addtogroup MISC
  * @{
  */
/* * @defgroup MISC_Exported_Types
  * @{
  */
/* * 
  * @brief  NVIC Init Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct NVIC_InitTypeDef {
    pub NVIC_IRQChannel: uint8_t,
    pub NVIC_IRQChannelPreemptionPriority: uint8_t,
    pub NVIC_IRQChannelSubPriority: uint8_t,
    pub NVIC_IRQChannelCmd: FunctionalState,
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
// microsecond time
pub type timeUs_t = uint32_t;
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
pub type rcc_reg = libc::c_uint;
pub const RCC_AHB1: rcc_reg = 4;
pub const RCC_APB1: rcc_reg = 3;
pub const RCC_APB2: rcc_reg = 2;
// make sure that default value (0) does not enable anything
pub const RCC_AHB: rcc_reg = 1;
pub const RCC_EMPTY: rcc_reg = 0;
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
pub type i2cHardware_t = i2cHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cHardware_s {
    pub device: I2CDevice,
    pub reg: *mut I2C_TypeDef,
    pub sclPins: [i2cPinDef_t; 4],
    pub sdaPins: [i2cPinDef_t; 4],
    pub rcc: rccPeriphTag_t,
    pub ev_irq: uint8_t,
    pub er_irq: uint8_t,
}
pub type i2cPinDef_t = i2cPinDef_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cPinDef_s {
    pub ioTag: ioTag_t,
}
pub type i2cDevice_t = i2cDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cDevice_s {
    pub hardware: *const i2cHardware_t,
    pub reg: *mut I2C_TypeDef,
    pub scl: IO_t,
    pub sda: IO_t,
    pub overClock: bool,
    pub pullUp: bool,
    pub state: i2cState_t,
}
pub type i2cState_t = i2cState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cState_s {
    pub error: bool,
    pub busy: bool,
    pub addr: uint8_t,
    pub reg: uint8_t,
    pub bytes: uint8_t,
    pub writing: uint8_t,
    pub reading: uint8_t,
    pub write_p: *mut uint8_t,
    pub read_p: *mut uint8_t,
}
#[inline(always)]
unsafe extern "C" fn __DMB() { asm!("dmb 0xF" : : : "memory" : "volatile"); }
// MCU/Driver dependent member follows
// Initialized in run_static_initializers
#[no_mangle]
pub static mut i2cHardware: [i2cHardware_t; 2] =
    [i2cHardware_t{device: I2CDEV_1,
                   reg: 0 as *mut I2C_TypeDef,
                   sclPins: [i2cPinDef_t{ioTag: 0,}; 4],
                   sdaPins: [i2cPinDef_t{ioTag: 0,}; 4],
                   rcc: 0,
                   ev_irq: 0,
                   er_irq: 0,}; 2];
#[no_mangle]
pub static mut i2cDevice: [i2cDevice_t; 2] =
    [i2cDevice_t{hardware: 0 as *const i2cHardware_t,
                 reg: 0 as *const I2C_TypeDef as *mut I2C_TypeDef,
                 scl: 0 as *const libc::c_void as *mut libc::c_void,
                 sda: 0 as *const libc::c_void as *mut libc::c_void,
                 overClock: false,
                 pullUp: false,
                 state:
                     i2cState_t{error: false,
                                busy: false,
                                addr: 0,
                                reg: 0,
                                bytes: 0,
                                writing: 0,
                                reading: 0,
                                write_p: 0 as *const uint8_t as *mut uint8_t,
                                read_p:
                                    0 as *const uint8_t as *mut uint8_t,},};
        2];
static mut i2cErrorCount: uint16_t = 0 as libc::c_int as uint16_t;
#[no_mangle]
pub unsafe extern "C" fn I2C1_ER_IRQHandler() { i2c_er_handler(I2CDEV_1); }
#[no_mangle]
pub unsafe extern "C" fn I2C1_EV_IRQHandler() { i2c_ev_handler(I2CDEV_1); }
#[no_mangle]
pub unsafe extern "C" fn I2C2_ER_IRQHandler() { i2c_er_handler(I2CDEV_2); }
#[no_mangle]
pub unsafe extern "C" fn I2C2_EV_IRQHandler() { i2c_ev_handler(I2CDEV_2); }
unsafe extern "C" fn i2cHandleHardwareFailure(mut device: I2CDevice) -> bool {
    ::core::ptr::write_volatile(&mut i2cErrorCount as *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&i2cErrorCount
                                                                           as
                                                                           *const uint16_t).wrapping_add(1));
    // reinit peripheral + clock out garbage
    i2cInit(device);
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn i2cWriteBuffer(mut device: I2CDevice,
                                        mut addr_: uint8_t, mut reg_: uint8_t,
                                        mut len_: uint8_t,
                                        mut data: *mut uint8_t) -> bool {
    if device as libc::c_int == I2CINVALID as libc::c_int ||
           device as libc::c_int > 2 as libc::c_int {
        return 0 as libc::c_int != 0
    }
    let mut I2Cx: *mut I2C_TypeDef = i2cDevice[device as usize].reg;
    if I2Cx.is_null() { return 0 as libc::c_int != 0 }
    let mut state: *mut i2cState_t =
        &mut (*i2cDevice.as_mut_ptr().offset(device as isize)).state;
    let mut timeout: uint32_t = 0x1000 as libc::c_int as uint32_t;
    ::core::ptr::write_volatile(&mut (*state).addr as *mut uint8_t,
                                ((addr_ as libc::c_int) << 1 as libc::c_int)
                                    as uint8_t);
    ::core::ptr::write_volatile(&mut (*state).reg as *mut uint8_t, reg_);
    ::core::ptr::write_volatile(&mut (*state).writing as *mut uint8_t,
                                1 as libc::c_int as uint8_t);
    ::core::ptr::write_volatile(&mut (*state).reading as *mut uint8_t,
                                0 as libc::c_int as uint8_t);
    (*state).write_p = data as *mut uint8_t;
    (*state).read_p = data as *mut uint8_t;
    ::core::ptr::write_volatile(&mut (*state).bytes as *mut uint8_t, len_);
    ::core::ptr::write_volatile(&mut (*state).busy as *mut bool,
                                1 as libc::c_int != 0);
    ::core::ptr::write_volatile(&mut (*state).error as *mut bool,
                                0 as libc::c_int != 0);
    if (*I2Cx).CR2 as libc::c_int &
           0x200 as libc::c_int as uint16_t as libc::c_int == 0 {
        // if we are restarting the driver
        if (*I2Cx).CR1 as libc::c_int &
               0x100 as libc::c_int as uint16_t as libc::c_int == 0 {
            // ensure sending a start
            while (*I2Cx).CR1 as libc::c_int &
                      0x200 as libc::c_int as uint16_t as libc::c_int != 0 &&
                      {
                          timeout =
                              timeout.wrapping_sub(1); // wait for any stop to finish sending
                          (timeout) > 0 as libc::c_int as libc::c_uint
                      } {
            }
            if timeout == 0 as libc::c_int as libc::c_uint {
                return i2cHandleHardwareFailure(device)
            }
            I2C_GenerateSTART(I2Cx, ENABLE);
            // send the start for the new job
        }
        I2C_ITConfig(I2Cx,
                     (0x200 as libc::c_int as uint16_t as libc::c_int |
                          0x100 as libc::c_int as uint16_t as libc::c_int) as
                         uint16_t, ENABLE);
        // allow the interrupts to fire off again
    }
    timeout = 0x1000 as libc::c_int as uint32_t;
    while (*state).busy as libc::c_int != 0 &&
              {
                  timeout = timeout.wrapping_sub(1);
                  (timeout) > 0 as libc::c_int as libc::c_uint
              } {
    }
    if timeout == 0 as libc::c_int as libc::c_uint {
        return i2cHandleHardwareFailure(device)
    }
    return !(*state).error;
}
#[no_mangle]
pub unsafe extern "C" fn i2cWrite(mut device: I2CDevice, mut addr_: uint8_t,
                                  mut reg_: uint8_t, mut data: uint8_t)
 -> bool {
    return i2cWriteBuffer(device, addr_, reg_, 1 as libc::c_int as uint8_t,
                          &mut data);
}
#[no_mangle]
pub unsafe extern "C" fn i2cRead(mut device: I2CDevice, mut addr_: uint8_t,
                                 mut reg_: uint8_t, mut len: uint8_t,
                                 mut buf: *mut uint8_t) -> bool {
    if device as libc::c_int == I2CINVALID as libc::c_int ||
           device as libc::c_int > 2 as libc::c_int {
        return 0 as libc::c_int != 0
    }
    let mut I2Cx: *mut I2C_TypeDef = i2cDevice[device as usize].reg;
    if I2Cx.is_null() { return 0 as libc::c_int != 0 }
    let mut state: *mut i2cState_t =
        &mut (*i2cDevice.as_mut_ptr().offset(device as isize)).state;
    let mut timeout: uint32_t = 0x1000 as libc::c_int as uint32_t;
    ::core::ptr::write_volatile(&mut (*state).addr as *mut uint8_t,
                                ((addr_ as libc::c_int) << 1 as libc::c_int)
                                    as uint8_t);
    ::core::ptr::write_volatile(&mut (*state).reg as *mut uint8_t, reg_);
    ::core::ptr::write_volatile(&mut (*state).writing as *mut uint8_t,
                                0 as libc::c_int as uint8_t);
    ::core::ptr::write_volatile(&mut (*state).reading as *mut uint8_t,
                                1 as libc::c_int as uint8_t);
    (*state).read_p = buf as *mut uint8_t;
    (*state).write_p = buf as *mut uint8_t;
    ::core::ptr::write_volatile(&mut (*state).bytes as *mut uint8_t, len);
    ::core::ptr::write_volatile(&mut (*state).busy as *mut bool,
                                1 as libc::c_int != 0);
    ::core::ptr::write_volatile(&mut (*state).error as *mut bool,
                                0 as libc::c_int != 0);
    if (*I2Cx).CR2 as libc::c_int &
           0x200 as libc::c_int as uint16_t as libc::c_int == 0 {
        // if we are restarting the driver
        if (*I2Cx).CR1 as libc::c_int &
               0x100 as libc::c_int as uint16_t as libc::c_int == 0 {
            // ensure sending a start
            while (*I2Cx).CR1 as libc::c_int &
                      0x200 as libc::c_int as uint16_t as libc::c_int != 0 &&
                      {
                          timeout =
                              timeout.wrapping_sub(1); // wait for any stop to finish sending
                          (timeout) > 0 as libc::c_int as libc::c_uint
                      } {
            }
            if timeout == 0 as libc::c_int as libc::c_uint {
                return i2cHandleHardwareFailure(device)
            }
            I2C_GenerateSTART(I2Cx, ENABLE);
            // send the start for the new job
        }
        I2C_ITConfig(I2Cx,
                     (0x200 as libc::c_int as uint16_t as libc::c_int |
                          0x100 as libc::c_int as uint16_t as libc::c_int) as
                         uint16_t, ENABLE);
        // allow the interrupts to fire off again
    }
    timeout = 0x1000 as libc::c_int as uint32_t;
    while (*state).busy as libc::c_int != 0 &&
              {
                  timeout = timeout.wrapping_sub(1);
                  (timeout) > 0 as libc::c_int as libc::c_uint
              } {
    }
    if timeout == 0 as libc::c_int as libc::c_uint {
        return i2cHandleHardwareFailure(device)
    }
    return !(*state).error;
}
unsafe extern "C" fn i2c_er_handler(mut device: I2CDevice) {
    let mut I2Cx: *mut I2C_TypeDef =
        (*i2cDevice[device as usize].hardware).reg;
    let mut state: *mut i2cState_t =
        &mut (*i2cDevice.as_mut_ptr().offset(device as isize)).state;
    // Read the I2C1 status register
    let mut SR1Register: uint32_t = (*I2Cx).SR1 as uint32_t;
    if SR1Register &
           (0x100 as libc::c_int as uint16_t as libc::c_int |
                0x200 as libc::c_int as uint16_t as libc::c_int |
                0x400 as libc::c_int as uint16_t as libc::c_int |
                0x800 as libc::c_int as uint16_t as libc::c_int) as
               libc::c_uint != 0 {
        // an error
        ::core::ptr::write_volatile(&mut (*state).error as *mut bool,
                                    1 as libc::c_int != 0)
    }
    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    if SR1Register &
           (0x100 as libc::c_int as uint16_t as libc::c_int |
                0x200 as libc::c_int as uint16_t as libc::c_int |
                0x400 as libc::c_int as uint16_t as libc::c_int) as
               libc::c_uint != 0 {
        // read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(I2Cx, 0x400 as libc::c_int as uint16_t,
                     DISABLE); // disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
        if SR1Register & 0x200 as libc::c_int as uint16_t as libc::c_uint == 0
               &&
               (*I2Cx).CR1 as libc::c_int &
                   0x200 as libc::c_int as uint16_t as libc::c_int == 0 {
            // if we dont have an ARLO error, ensure sending of a stop
            if (*I2Cx).CR1 as libc::c_int &
                   0x100 as libc::c_int as uint16_t as libc::c_int != 0 {
                // We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
                while (*I2Cx).CR1 as libc::c_int &
                          0x100 as libc::c_int as uint16_t as libc::c_int != 0
                      {
                } // wait for any start to finish sending
                // reset and configure the hardware
                I2C_GenerateSTOP(I2Cx,
                                 ENABLE); // send stop to finalise bus transaction
                while (*I2Cx).CR1 as libc::c_int &
                          0x200 as libc::c_int as uint16_t as libc::c_int != 0
                      {
                } // wait for stop to finish sending
                i2cInit(device); // stop to free up the bus
            } else {
                I2C_GenerateSTOP(I2Cx, ENABLE);
                I2C_ITConfig(I2Cx,
                             (0x200 as libc::c_int as uint16_t as libc::c_int
                                  |
                                  0x100 as libc::c_int as uint16_t as
                                      libc::c_int) as uint16_t, DISABLE);
                // Disable EVT and ERR interrupts while bus inactive
            }
        }
    } // reset all the error bits to clear the interrupt
    ::core::ptr::write_volatile(&mut (*I2Cx).SR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).SR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(0x100 as libc::c_int as uint16_t as
                                           libc::c_int |
                                           0x200 as libc::c_int as uint16_t as
                                               libc::c_int |
                                           0x400 as libc::c_int as uint16_t as
                                               libc::c_int |
                                           0x800 as libc::c_int as uint16_t as
                                               libc::c_int)) as uint16_t as
                                    uint16_t); // flag to indicate if subaddess sent, flag to indicate final bus condition
    ::core::ptr::write_volatile(&mut (*state).busy as *mut bool,
                                0 as libc::c_int !=
                                    0); // index is signed -1 == send the subaddress
}
unsafe extern "C" fn i2c_ev_handler(mut device: I2CDevice) {
    let mut I2Cx: *mut I2C_TypeDef =
        (*i2cDevice[device as
                        usize].hardware).reg; // read the status register here
    let mut state: *mut i2cState_t =
        &mut (*i2cDevice.as_mut_ptr().offset(device as isize)).state;
    static mut subaddress_sent: uint8_t = 0;
    static mut final_stop: uint8_t = 0;
    static mut index: int8_t = 0;
    let mut SReg_1: uint8_t = (*I2Cx).SR1 as uint8_t;
    if SReg_1 as libc::c_int & 0x1 as libc::c_int as uint16_t as libc::c_int
           != 0 {
        // we just sent a start - EV5 in ref manual
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x800 as libc::c_int as uint16_t as
                                               libc::c_int)) as uint16_t as
                                        uint16_t); // reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(I2Cx, ENABLE); // make sure ACK is on
        index = 0 as libc::c_int as int8_t; // reset the index
        if (*state).reading as libc::c_int != 0 &&
               (subaddress_sent as libc::c_int != 0 ||
                    0xff as libc::c_int == (*state).reg as libc::c_int) {
            // we have sent the subaddr
            subaddress_sent =
                1 as libc::c_int as
                    uint8_t; // make sure this is set in case of no subaddress, so following code runs correctly
            // send the address and set hardware mode
            if (*state).bytes as libc::c_int == 2 as libc::c_int {
                ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                            (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                        as
                                                                                        *const uint16_t)
                                                 as libc::c_int |
                                                 0x800 as libc::c_int as
                                                     uint16_t as libc::c_int)
                                                as uint16_t as uint16_t)
            } // set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(I2Cx, (*state).addr,
                                0x1 as libc::c_int as uint8_t);
        } else {
            // direction is Tx, or we havent sent the sub and rep start
            I2C_Send7bitAddress(I2Cx, (*state).addr,
                                0 as libc::c_int as uint8_t);
            if (*state).reg as libc::c_int != 0xff as libc::c_int
               { // send the address and set hardware mode
                // send a subaddress
                // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
                index = -(1 as libc::c_int) as int8_t
            }
        }
    } else if SReg_1 as libc::c_int &
                  0x2 as libc::c_int as uint16_t as libc::c_int != 0 {
        // we just sent the address - EV6 in ref manual
        // Read SR1,2 to clear ADDR
        __DMB(); // memory fence to control hardware
        if (*state).bytes as libc::c_int == 1 as libc::c_int &&
               (*state).reading as libc::c_int != 0 &&
               subaddress_sent as libc::c_int != 0 {
            // we are receiving 1 byte - EV6_3
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
            __DMB(); // turn off ACK
            // allow us to have an EV7
            // clear ADDR after ACK is turned off
            I2C_GenerateSTOP(I2Cx, ENABLE); // program the stop
            final_stop = 1 as libc::c_int as uint8_t;
            I2C_ITConfig(I2Cx, 0x400 as libc::c_int as uint16_t, ENABLE);
        } else {
            // EV6 and EV6_1
            // clear the ADDR here
            __DMB();
            if (*state).bytes as libc::c_int == 2 as libc::c_int &&
                   (*state).reading as libc::c_int != 0 &&
                   subaddress_sent as libc::c_int != 0 {
                // rx 2 bytes - EV6_1
                I2C_AcknowledgeConfig(I2Cx, DISABLE);
                I2C_ITConfig(I2Cx, 0x400 as libc::c_int as uint16_t,
                             DISABLE); // turn off ACK
                // disable TXE to allow the buffer to fill
            } else if (*state).bytes as libc::c_int == 3 as libc::c_int &&
                          (*state).reading as libc::c_int != 0 &&
                          subaddress_sent as libc::c_int != 0 {
                // rx 3 bytes
                I2C_ITConfig(I2Cx, 0x400 as libc::c_int as uint16_t,
                             DISABLE); // make sure RXNE disabled so we get a BTF in two bytes time
            } else {
                // receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(I2Cx, 0x400 as libc::c_int as uint16_t, ENABLE);
            }
        }
    } else if SReg_1 as libc::c_int &
                  0x4 as libc::c_int as uint16_t as libc::c_int != 0 {
        // Byte transfer finished - EV7_2, EV7_3 or EV8_2
        final_stop = 1 as libc::c_int as uint8_t;
        if (*state).reading as libc::c_int != 0 &&
               subaddress_sent as libc::c_int != 0 {
            // EV7_2, EV7_3
            if (*state).bytes as libc::c_int > 2 as libc::c_int {
                // EV7_2
                I2C_AcknowledgeConfig(I2Cx, DISABLE); // turn off ACK
                // enable TXE to allow the final EV7
                let fresh0 = index; // read data N-2
                index = index + 1; // program the Stop
                ::core::ptr::write_volatile((*state).read_p.offset(fresh0 as
                                                                       isize),
                                            (*I2Cx).DR as
                                                uint8_t); // required to fix hardware
                I2C_GenerateSTOP(I2Cx, ENABLE); // read data N - 1
                final_stop = 1 as libc::c_int as uint8_t;
                let fresh1 = index;
                index = index + 1;
                ::core::ptr::write_volatile((*state).read_p.offset(fresh1 as
                                                                       isize),
                                            (*I2Cx).DR as uint8_t);
                I2C_ITConfig(I2Cx, 0x400 as libc::c_int as uint16_t, ENABLE);
            } else {
                // EV7_3
                if final_stop != 0 { // program a rep start
                    I2C_GenerateSTOP(I2Cx, ENABLE); // program the Stop
                } else { I2C_GenerateSTART(I2Cx, ENABLE); }
                // to show job completed
                let fresh2 = index; // read data N - 1
                index = index + 1; // read data N
                ::core::ptr::write_volatile((*state).read_p.offset(fresh2 as
                                                                       isize),
                                            (*I2Cx).DR as uint8_t);
                let fresh3 = index;
                index = index + 1;
                ::core::ptr::write_volatile((*state).read_p.offset(fresh3 as
                                                                       isize),
                                            (*I2Cx).DR as uint8_t);
                index += 1
            }
        } else if subaddress_sent as libc::c_int != 0 ||
                      (*state).writing as libc::c_int != 0 {
            // EV8_2, which may be due to a subaddress sent or a write completion
            if final_stop != 0 { // program a rep start
                I2C_GenerateSTOP(I2Cx, ENABLE); // program the Stop
            } else { I2C_GenerateSTART(I2Cx, ENABLE); }
            index += 1
            // to show that the job is complete
        } else {
            // We need to send a subaddress
            I2C_GenerateSTART(I2Cx, ENABLE);
            subaddress_sent =
                1 as libc::c_int as uint8_t // program the repeated Start
            // this is set back to zero upon completion of the current task
        }
        // we must wait for the start to clear, otherwise we get constant BTF
        while (*I2Cx).CR1 as libc::c_int &
                  0x100 as libc::c_int as uint16_t as libc::c_int != 0 {
        }
    } else if SReg_1 as libc::c_int &
                  0x40 as libc::c_int as uint16_t as libc::c_int != 0 {
        // Byte received - EV7
        let fresh4 = index;
        index = index + 1;
        ::core::ptr::write_volatile((*state).read_p.offset(fresh4 as isize),
                                    (*I2Cx).DR as uint8_t);
        // to show job is complete
        if (*state).bytes as libc::c_int ==
               index as libc::c_int + 3 as libc::c_int {
            I2C_ITConfig(I2Cx, 0x400 as libc::c_int as uint16_t,
                         DISABLE); // disable TXE to allow the buffer to flush so we can get an EV7_2
        }
        if (*state).bytes as libc::c_int == index as libc::c_int {
            // We have completed a final EV7
            index += 1
        }
    } else if SReg_1 as libc::c_int &
                  0x80 as libc::c_int as uint16_t as libc::c_int != 0 {
        // Byte transmitted EV8 / EV8_1
        if index as libc::c_int != -(1 as libc::c_int) {
            // we dont have a subaddress to send
            let fresh5 = index;
            index = index + 1;
            ::core::ptr::write_volatile(&mut (*I2Cx).DR as *mut uint16_t,
                                        *(*state).write_p.offset(fresh5 as
                                                                     isize) as
                                            uint16_t);
            if (*state).bytes as libc::c_int == index as libc::c_int {
                // disable TXE to allow the buffer to flush
                // we have sent all the data
                I2C_ITConfig(I2Cx, 0x400 as libc::c_int as uint16_t, DISABLE);
            }
        } else {
            index += 1;
            // disable TXE to allow the buffer to flush
            ::core::ptr::write_volatile(&mut (*I2Cx).DR as *mut uint16_t,
                                        (*state).reg as
                                            uint16_t); // send the subaddress
            if (*state).reading as libc::c_int != 0 || (*state).bytes == 0 {
                // if receiving or sending 0 bytes, flush now
                I2C_ITConfig(I2Cx, 0x400 as libc::c_int as uint16_t, DISABLE);
            }
        }
    }
    if index as libc::c_int ==
           (*state).bytes as libc::c_int + 1 as libc::c_int {
        // we have completed the current job
        subaddress_sent = 0 as libc::c_int as uint8_t; // reset this here
        if final_stop != 0
           { // Disable EVT and ERR interrupts while bus inactive
            // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(I2Cx,
                         (0x200 as libc::c_int as uint16_t as libc::c_int |
                              0x100 as libc::c_int as uint16_t as libc::c_int)
                             as uint16_t, DISABLE);
        }
        ::core::ptr::write_volatile(&mut (*state).busy as *mut bool,
                                    0 as libc::c_int != 0)
    };
}
#[no_mangle]
pub unsafe extern "C" fn i2cInit(mut device: I2CDevice) {
    if device as libc::c_int == I2CINVALID as libc::c_int { return }
    let mut pDev: *mut i2cDevice_t =
        &mut *i2cDevice.as_mut_ptr().offset(device as isize) as
            *mut i2cDevice_t;
    let mut hw: *const i2cHardware_t = (*pDev).hardware;
    if hw.is_null() { return }
    let mut I2Cx: *mut I2C_TypeDef = (*hw).reg;
    memset(&mut (*pDev).state as *mut i2cState_t as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<i2cState_t>() as libc::c_ulong);
    let mut nvic: NVIC_InitTypeDef =
        NVIC_InitTypeDef{NVIC_IRQChannel: 0,
                         NVIC_IRQChannelPreemptionPriority: 0,
                         NVIC_IRQChannelSubPriority: 0,
                         NVIC_IRQChannelCmd: DISABLE,};
    let mut i2cInit_0: I2C_InitTypeDef =
        I2C_InitTypeDef{I2C_ClockSpeed: 0,
                        I2C_Mode: 0,
                        I2C_DutyCycle: 0,
                        I2C_OwnAddress1: 0,
                        I2C_Ack: 0,
                        I2C_AcknowledgedAddress: 0,};
    let mut scl: IO_t = (*pDev).scl;
    let mut sda: IO_t = (*pDev).sda;
    IOInit(scl, OWNER_I2C_SCL,
           (device as libc::c_int + 1 as libc::c_int) as uint8_t);
    IOInit(sda, OWNER_I2C_SDA,
           (device as libc::c_int + 1 as libc::c_int) as uint8_t);
    // Enable RCC
    RCC_ClockCmd((*hw).rcc, ENABLE);
    I2C_ITConfig(I2Cx,
                 (0x200 as libc::c_int as uint16_t as libc::c_int |
                      0x100 as libc::c_int as uint16_t as libc::c_int) as
                     uint16_t, DISABLE);
    i2cUnstick(scl, sda);
    // Init pins
    IOConfigGPIO(scl,
                 (GPIO_Mode_AF_OD as libc::c_int |
                      GPIO_Speed_50MHz as libc::c_int) as
                     ioConfig_t); // Disable EVT and ERR interrupts - they are enabled by the first request
    IOConfigGPIO(sda,
                 (GPIO_Mode_AF_OD as libc::c_int |
                      GPIO_Speed_50MHz as libc::c_int) as ioConfig_t);
    I2C_DeInit(I2Cx);
    I2C_StructInit(&mut i2cInit_0);
    I2C_ITConfig(I2Cx,
                 (0x200 as libc::c_int as uint16_t as libc::c_int |
                      0x100 as libc::c_int as uint16_t as libc::c_int) as
                     uint16_t, DISABLE);
    i2cInit_0.I2C_Mode = 0 as libc::c_int as uint16_t;
    i2cInit_0.I2C_DutyCycle = 0xbfff as libc::c_int as uint16_t;
    i2cInit_0.I2C_OwnAddress1 = 0 as libc::c_int as uint16_t;
    i2cInit_0.I2C_Ack = 0x400 as libc::c_int as uint16_t;
    i2cInit_0.I2C_AcknowledgedAddress = 0x4000 as libc::c_int as uint16_t;
    if (*pDev).overClock {
        i2cInit_0.I2C_ClockSpeed = 800000 as libc::c_int as uint32_t
        // 800khz Maximum speed tested on various boards without issues
    } else {
        i2cInit_0.I2C_ClockSpeed = 400000 as libc::c_int as uint32_t
        // 400khz Operation according specs
    }
    I2C_Cmd(I2Cx, ENABLE);
    I2C_Init(I2Cx, &mut i2cInit_0);
    I2C_StretchClockCmd(I2Cx, ENABLE);
    // I2C ER Interrupt
    nvic.NVIC_IRQChannel = (*hw).er_irq;
    nvic.NVIC_IRQChannelPreemptionPriority =
        ((((0 as libc::c_int) <<
               (4 as libc::c_int as
                    libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                    libc::c_uint).wrapping_sub(0x500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint32_t
                                                                                   >>
                                                                                   8
                                                                                       as
                                                                                       libc::c_int))
               |
               0 as libc::c_int &
                   0xf as libc::c_int >>
                       (7 as libc::c_int as
                            libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                           uint32_t >>
                                                           8 as libc::c_int))
              << 4 as libc::c_int & 0xf0 as libc::c_int) >>
             (4 as libc::c_int as
                  libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                  libc::c_uint).wrapping_sub(0x500
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 uint32_t
                                                                                 >>
                                                                                 8
                                                                                     as
                                                                                     libc::c_int))
             >> 4 as libc::c_int) as uint8_t;
    nvic.NVIC_IRQChannelSubPriority =
        ((((0 as libc::c_int) <<
               (4 as libc::c_int as
                    libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                    libc::c_uint).wrapping_sub(0x500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint32_t
                                                                                   >>
                                                                                   8
                                                                                       as
                                                                                       libc::c_int))
               |
               0 as libc::c_int &
                   0xf as libc::c_int >>
                       (7 as libc::c_int as
                            libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                           uint32_t >>
                                                           8 as libc::c_int))
              << 4 as libc::c_int & 0xf0 as libc::c_int &
              0xf as libc::c_int >>
                  (7 as libc::c_int as
                       libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                      uint32_t >>
                                                      8 as libc::c_int)) >>
             4 as libc::c_int) as uint8_t;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&mut nvic);
    // I2C EV Interrupt
    nvic.NVIC_IRQChannel = (*hw).ev_irq;
    nvic.NVIC_IRQChannelPreemptionPriority =
        ((((0 as libc::c_int) <<
               (4 as libc::c_int as
                    libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                    libc::c_uint).wrapping_sub(0x500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint32_t
                                                                                   >>
                                                                                   8
                                                                                       as
                                                                                       libc::c_int))
               |
               0 as libc::c_int &
                   0xf as libc::c_int >>
                       (7 as libc::c_int as
                            libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                           uint32_t >>
                                                           8 as libc::c_int))
              << 4 as libc::c_int & 0xf0 as libc::c_int) >>
             (4 as libc::c_int as
                  libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                  libc::c_uint).wrapping_sub(0x500
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 uint32_t
                                                                                 >>
                                                                                 8
                                                                                     as
                                                                                     libc::c_int))
             >> 4 as libc::c_int) as uint8_t;
    nvic.NVIC_IRQChannelSubPriority =
        ((((0 as libc::c_int) <<
               (4 as libc::c_int as
                    libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                    libc::c_uint).wrapping_sub(0x500
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint32_t
                                                                                   >>
                                                                                   8
                                                                                       as
                                                                                       libc::c_int))
               |
               0 as libc::c_int &
                   0xf as libc::c_int >>
                       (7 as libc::c_int as
                            libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                           uint32_t >>
                                                           8 as libc::c_int))
              << 4 as libc::c_int & 0xf0 as libc::c_int &
              0xf as libc::c_int >>
                  (7 as libc::c_int as
                       libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                      uint32_t >>
                                                      8 as libc::c_int)) >>
             4 as libc::c_int) as uint8_t;
    NVIC_Init(&mut nvic);
}
#[no_mangle]
pub unsafe extern "C" fn i2cGetErrorCounter() -> uint16_t {
    return i2cErrorCount;
}
unsafe extern "C" fn i2cUnstick(mut scl: IO_t, mut sda: IO_t) {
    let mut i: libc::c_int = 0;
    IOHi(scl);
    IOHi(sda);
    IOConfigGPIO(scl,
                 (GPIO_Mode_Out_OD as libc::c_int |
                      GPIO_Speed_2MHz as libc::c_int) as ioConfig_t);
    IOConfigGPIO(sda,
                 (GPIO_Mode_Out_OD as libc::c_int |
                      GPIO_Speed_2MHz as libc::c_int) as ioConfig_t);
    // Clock out, with SDA high:
    //   7 data bits
    //   1 READ bit
    //   1 cycle for the ACK
    i = 0 as libc::c_int;
    while i < 7 as libc::c_int + 1 as libc::c_int + 1 as libc::c_int {
        // Wait for any clock stretching to finish
        let mut timeout: libc::c_int = 500 as libc::c_int / 10 as libc::c_int;
        while !IORead(scl) && timeout != 0 {
            delayMicroseconds(10 as libc::c_int as timeUs_t);
            timeout -= 1
        }
        // Pull low
        IOLo(scl); // Set bus low
        delayMicroseconds((10 as libc::c_int / 2 as libc::c_int) as
                              timeUs_t); // Set bus high
        IOHi(scl);
        delayMicroseconds((10 as libc::c_int / 2 as libc::c_int) as timeUs_t);
        i += 1
    }
    // Generate a stop condition in case there was none
    IOLo(scl); // Set bus scl high
    delayMicroseconds((10 as libc::c_int / 2 as libc::c_int) as timeUs_t);
    IOLo(sda);
    delayMicroseconds((10 as libc::c_int / 2 as libc::c_int) as timeUs_t);
    IOHi(scl);
    delayMicroseconds((10 as libc::c_int / 2 as libc::c_int) as timeUs_t);
    IOHi(sda);
    // Set bus sda high
}
unsafe extern "C" fn run_static_initializers() {
    i2cHardware =
        [{
             let mut init =
                 i2cHardware_s{device: I2CDEV_2,
                               reg:
                                   (0x40000000 as libc::c_int as
                                        uint32_t).wrapping_add(0x5800 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_uint)
                                       as *mut I2C_TypeDef,
                               sclPins:
                                   [{
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 10 as
                                                                     libc::c_int)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            0 as libc::c_int
                                                                as ioTag_t,};
                                        init
                                    }, i2cPinDef_t{ioTag: 0,},
                                    i2cPinDef_t{ioTag: 0,}],
                               sdaPins:
                                   [{
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 11 as
                                                                     libc::c_int)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            0 as libc::c_int
                                                                as ioTag_t,};
                                        init
                                    }, i2cPinDef_t{ioTag: 0,},
                                    i2cPinDef_t{ioTag: 0,}],
                               rcc:
                                   (((RCC_APB1 as libc::c_int) <<
                                         5 as libc::c_int) as libc::c_long |
                                        (16 as libc::c_int *
                                             (0x400000 as libc::c_int as
                                                  uint32_t as libc::c_long >
                                                  65535 as libc::c_long) as
                                                 libc::c_int) as libc::c_long
                                            +
                                            ((8 as libc::c_int *
                                                  (0x400000 as libc::c_int as
                                                       uint32_t as
                                                       libc::c_long *
                                                       1 as libc::c_long >>
                                                       16 as libc::c_int *
                                                           (0x400000 as
                                                                libc::c_int as
                                                                uint32_t as
                                                                libc::c_long >
                                                                65535 as
                                                                    libc::c_long)
                                                               as libc::c_int
                                                       >
                                                       255 as libc::c_int as
                                                           libc::c_long) as
                                                      libc::c_int) as
                                                 libc::c_long +
                                                 (8 as libc::c_int as
                                                      libc::c_long -
                                                      90 as libc::c_int as
                                                          libc::c_long /
                                                          ((0x400000 as
                                                                libc::c_int as
                                                                uint32_t as
                                                                libc::c_long *
                                                                1 as
                                                                    libc::c_long
                                                                >>
                                                                16 as
                                                                    libc::c_int
                                                                    *
                                                                    (0x400000
                                                                         as
                                                                         libc::c_int
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8 as
                                                                    libc::c_int
                                                                    *
                                                                    (0x400000
                                                                         as
                                                                         libc::c_int
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1 as
                                                                             libc::c_long
                                                                         >>
                                                                         16 as
                                                                             libc::c_int
                                                                             *
                                                                             (0x400000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  uint32_t
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535
                                                                                      as
                                                                                      libc::c_long)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               4 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               +
                                                               14 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               |
                                                               1 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long)
                                                      -
                                                      2 as libc::c_int as
                                                          libc::c_long /
                                                          ((0x400000 as
                                                                libc::c_int as
                                                                uint32_t as
                                                                libc::c_long *
                                                                1 as
                                                                    libc::c_long
                                                                >>
                                                                16 as
                                                                    libc::c_int
                                                                    *
                                                                    (0x400000
                                                                         as
                                                                         libc::c_int
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8 as
                                                                    libc::c_int
                                                                    *
                                                                    (0x400000
                                                                         as
                                                                         libc::c_int
                                                                         as
                                                                         uint32_t
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1 as
                                                                             libc::c_long
                                                                         >>
                                                                         16 as
                                                                             libc::c_int
                                                                             *
                                                                             (0x400000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  uint32_t
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535
                                                                                      as
                                                                                      libc::c_long)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               2 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               +
                                                               1 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long))))
                                       as rccPeriphTag_t,
                               ev_irq: I2C2_EV_IRQn as libc::c_int as uint8_t,
                               er_irq:
                                   I2C2_ER_IRQn as libc::c_int as uint8_t,};
             init
         },
         i2cHardware_t{device: I2CDEV_1,
                       reg: 0 as *mut I2C_TypeDef,
                       sclPins: [i2cPinDef_t{ioTag: 0,}; 4],
                       sdaPins: [i2cPinDef_t{ioTag: 0,}; 4],
                       rcc: 0,
                       ev_irq: 0,
                       er_irq: 0,}]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
