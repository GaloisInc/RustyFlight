use ::libc;
extern "C" {
    #[no_mangle]
    fn GetEPTxAddr(_: uint8_t) -> uint16_t;
    #[no_mangle]
    fn GetEPRxAddr(_: uint8_t) -> uint16_t;
    #[no_mangle]
    fn SetEPTxCount(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn GetEPRxCount(_: uint8_t) -> uint16_t;
    #[no_mangle]
    static mut wInterrupt_Mask: uint16_t;
    /* *
  ******************************************************************************
  * @file    usb_mem.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Utility prototypes functions for memory/PMA transfers
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
    fn UserToPMABufferCopy(pbUsrBuf: *const uint8_t, wPMABufAddr: uint16_t,
                           wNBytes: uint16_t);
    #[no_mangle]
    fn PMAToUserBufferCopy(pbUsrBuf: *mut uint8_t, wPMABufAddr: uint16_t,
                           wNBytes: uint16_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  ******************************************************************************
  * @file    usb_sil.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Simplified Interface Layer for Global Initialization and Endpoint
  *          Rea/Write operations.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ******************************************************************************
* Function Name  : USB_SIL_Init
* Description    : Initialize the USB Device IP and the Endpoint 0.
* Input          : None.
* Output         : None.
* Return         : Status.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn USB_SIL_Init() -> uint32_t {
    /* USB interrupts initialization */
  /* clear pending interrupts */
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x44 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                0 as libc::c_int as uint16_t as libc::c_uint);
    wInterrupt_Mask =
        (0x8000 as libc::c_int | 0x1000 as libc::c_int | 0x200 as libc::c_int
             | 0x400 as libc::c_int) as uint16_t;
    /* set interrupts mask */
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                wInterrupt_Mask as libc::c_uint);
    return 0 as libc::c_int as uint32_t;
}
/* ******************************************************************************
* Function Name  : USB_SIL_Write
* Description    : Write a buffer of data to a selected endpoint.
* Input          : - bEpAddr: The address of the non control endpoint.
*                  - pBufferPointer: The pointer to the buffer of data to be written
*                    to the endpoint.
*                  - wBufferSize: Number of data to be written (in bytes).
* Output         : None.
* Return         : Status.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn USB_SIL_Write(mut bEpAddr: uint8_t,
                                       mut pBufferPointer: *mut uint8_t,
                                       mut wBufferSize: uint32_t)
 -> uint32_t {
    /* Use the memory interface function to write to the selected endpoint */
    UserToPMABufferCopy(pBufferPointer,
                        GetEPTxAddr((bEpAddr as libc::c_int &
                                         0x7f as libc::c_int) as uint8_t),
                        wBufferSize as uint16_t);
    /* Update the data length in the control register */
    SetEPTxCount((bEpAddr as libc::c_int & 0x7f as libc::c_int) as uint8_t,
                 wBufferSize as uint16_t);
    return 0 as libc::c_int as uint32_t;
}
/* *
  ******************************************************************************
  * @file    usb_sil.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Simplified Interface Layer function prototypes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
/* ******************************************************************************
* Function Name  : USB_SIL_Read
* Description    : Write a buffer of data to a selected endpoint.
* Input          : - bEpAddr: The address of the non control endpoint.
*                  - pBufferPointer: The pointer to which will be saved the 
*                     received data buffer.
* Output         : None.
* Return         : Number of received data (in Bytes).
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn USB_SIL_Read(mut bEpAddr: uint8_t,
                                      mut pBufferPointer: *mut uint8_t)
 -> uint32_t {
    let mut DataLength: uint32_t = 0 as libc::c_int as uint32_t;
    /* Get the number of received data on the selected Endpoint */
    DataLength =
        GetEPRxCount((bEpAddr as libc::c_int & 0x7f as libc::c_int) as
                         uint8_t) as uint32_t;
    /* Use the memory interface function to write to the selected endpoint */
    PMAToUserBufferCopy(pBufferPointer,
                        GetEPRxAddr((bEpAddr as libc::c_int &
                                         0x7f as libc::c_int) as uint8_t),
                        DataLength as uint16_t);
    /* Return the number of received data */
    return DataLength;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
