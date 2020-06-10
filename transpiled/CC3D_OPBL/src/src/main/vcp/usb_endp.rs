use ::libc;
extern "C" {
    /* External variables --------------------------------------------------------*/
    #[no_mangle]
    static mut packetSent: uint32_t;
    #[no_mangle]
    fn GetEPRxCount(_: uint8_t) -> uint16_t;
    #[no_mangle]
    fn PMAToUserBufferCopy(pbUsrBuf: *mut uint8_t, wPMABufAddr: uint16_t,
                           wNBytes: uint16_t);
    /* *
 ******************************************************************************
 * @file    usb_endp.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Endpoint routines
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
    /* Interval between sending IN packets in frame number (1 frame = 1ms) */
    /* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
    // HJI
    #[no_mangle]
    static mut receiveBuffer: [uint8_t; 64];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
// HJI
#[no_mangle]
pub static mut receiveLength: uint32_t = 0;
/* function prototypes Automatically built defining related macros */
// HJI
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ******************************************************************************
 * Function Name  : EP1_IN_Callback
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn EP1_IN_Callback() {
    ::core::ptr::write_volatile(&mut packetSent as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    // HJI
}
/* ******************************************************************************
 * Function Name  : EP3_OUT_Callback
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn EP3_OUT_Callback() {
    ::core::ptr::write_volatile(&mut receiveLength as *mut uint32_t,
                                GetEPRxCount(3 as libc::c_int as uint8_t) as
                                    uint32_t); // HJI
    PMAToUserBufferCopy(receiveBuffer.as_mut_ptr() as *mut libc::c_uchar,
                        0x110 as libc::c_int as uint16_t,
                        receiveLength as uint16_t);
    // HJI
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
