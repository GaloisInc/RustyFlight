use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  ******************************************************************************
  * @file    usb_mem.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Utility functions for memory transfers to/from PMA
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
* Function Name  : UserToPMABufferCopy
* Description    : Copy a buffer from user memory area to packet memory area (PMA)
* Input          : - pbUsrBuf: pointer to user memory area.
*                  - wPMABufAddr: address into PMA.
*                  - wNBytes: no. of bytes to be copied.
* Output         : None.
* Return         : None	.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn UserToPMABufferCopy(mut pbUsrBuf: *const uint8_t,
                                             mut wPMABufAddr: uint16_t,
                                             mut wNBytes: uint16_t) {
    let mut n: uint32_t =
        (wNBytes as libc::c_int + 1 as libc::c_int >> 1 as libc::c_int) as
            uint32_t; /* n = (wNBytes + 1) / 2 */
    let mut i: uint32_t = 0;
    let mut temp1: uint32_t = 0;
    let mut temp2: uint32_t = 0;
    let mut pdwVal: *mut uint16_t = 0 as *mut uint16_t;
    pdwVal =
        ((wPMABufAddr as libc::c_int * 2 as libc::c_int) as libc::c_long +
             0x40006000 as libc::c_long) as *mut uint16_t;
    i = n;
    while i != 0 as libc::c_int as libc::c_uint {
        temp1 = *pbUsrBuf as uint16_t as uint32_t;
        pbUsrBuf = pbUsrBuf.offset(1);
        temp2 =
            temp1 |
                ((*pbUsrBuf as uint16_t as libc::c_int) << 8 as libc::c_int)
                    as libc::c_uint;
        let fresh0 = pdwVal;
        pdwVal = pdwVal.offset(1);
        *fresh0 = temp2 as uint16_t;
        pdwVal = pdwVal.offset(1);
        pbUsrBuf = pbUsrBuf.offset(1);
        i = i.wrapping_sub(1)
    };
}
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
/* ******************************************************************************
* Function Name  : PMAToUserBufferCopy
* Description    : Copy a buffer from user memory area to packet memory area (PMA)
* Input          : - pbUsrBuf    = pointer to user memory area.
*                  - wPMABufAddr = address into PMA.
*                  - wNBytes     = no. of bytes to be copied.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn PMAToUserBufferCopy(mut pbUsrBuf: *mut uint8_t,
                                             mut wPMABufAddr: uint16_t,
                                             mut wNBytes: uint16_t) {
    let mut n: uint32_t =
        (wNBytes as libc::c_int + 1 as libc::c_int >> 1 as libc::c_int) as
            uint32_t; /* /2*/
    let mut i: uint32_t = 0;
    let mut pdwVal: *mut uint32_t = 0 as *mut uint32_t;
    pdwVal =
        ((wPMABufAddr as libc::c_int * 2 as libc::c_int) as libc::c_long +
             0x40006000 as libc::c_long) as *mut uint32_t;
    i = n;
    while i != 0 as libc::c_int as libc::c_uint {
        let fresh1 = pdwVal;
        pdwVal = pdwVal.offset(1);
        let fresh2 = pbUsrBuf;
        pbUsrBuf = pbUsrBuf.offset(1);
        *(fresh2 as *mut uint16_t) = *fresh1 as uint16_t;
        pbUsrBuf = pbUsrBuf.offset(1);
        i = i.wrapping_sub(1)
    };
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
