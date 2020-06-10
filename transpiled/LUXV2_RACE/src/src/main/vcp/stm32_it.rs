use ::libc;
extern "C" {
    #[no_mangle]
    fn EXTI_ClearITPendingBit(EXTI_Line: uint32_t);
    /* *
 ******************************************************************************
 * @file    usb_istr.h
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   This file includes the peripherals header files in the user application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
    /* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
    #[no_mangle]
    fn USB_Istr();
}
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
/* *
 ******************************************************************************
 * @file    stm32_it.h
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   This file contains the headers of the interrupt handlers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* *
 ******************************************************************************
 * @file    stm32_it.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and peripherals
 *          interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* *****************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                         */
/* *****************************************************************************/
/* ******************************************************************************
 * Function Name  : NMI_Handler
 * Description    : This function handles NMI exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn NMI_Handler() { }
/* ******************************************************************************
 * Function Name  : MemManage_Handler
 * Description    : This function handles Memory Manage exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn MemManage_Handler() {
    loop 
         /* Go to infinite loop when Memory Manage exception occurs */
         {
    };
}
/* ******************************************************************************
 * Function Name  : BusFault_Handler
 * Description    : This function handles Bus Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn BusFault_Handler() {
    loop 
         /* Go to infinite loop when Bus Fault exception occurs */
         {
    };
}
/* ******************************************************************************
 * Function Name  : UsageFault_Handler
 * Description    : This function handles Usage Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn UsageFault_Handler() {
    loop 
         /* Go to infinite loop when Usage Fault exception occurs */
         {
    };
}
/* ******************************************************************************
 * Function Name  : SVC_Handler
 * Description    : This function handles SVCall exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SVC_Handler() { }
/* ******************************************************************************
 * Function Name  : DebugMon_Handler
 * Description    : This function handles Debug Monitor exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn DebugMon_Handler() { }
/* ******************************************************************************
 * Function Name  : PendSV_Handler
 * Description    : This function handles PendSVC exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn PendSV_Handler() { }
/* ******************************************************************************
 * Function Name  : USB_IRQHandler
 * Description    : This function handles USB Low Priority interrupts
 *                  requests.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn USB_LP_CAN1_RX0_IRQHandler() { USB_Istr(); }
/* ******************************************************************************
 * Function Name  : USB_FS_WKUP_IRQHandler
 * Description    : This function handles USB WakeUp interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn USBWakeUp_IRQHandler() {
    EXTI_ClearITPendingBit(0x12 as libc::c_int as uint32_t);
}
