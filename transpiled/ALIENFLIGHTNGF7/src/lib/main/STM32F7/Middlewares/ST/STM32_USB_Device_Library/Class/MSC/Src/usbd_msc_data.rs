use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type uint8_t = __uint8_t;
/* *
  ******************************************************************************
  * @file    usbd_msc_data.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides all the vital inquiry pages and sense data.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
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
/* * @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
/* * @defgroup MSC_DATA 
  * @brief Mass storage info/data module
  * @{
  */
/* * @defgroup MSC_DATA_Private_TypesDefinitions
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup MSC_DATA_Private_Defines
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup MSC_DATA_Private_Macros
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup MSC_DATA_Private_Variables
  * @{
  */
/* USB Mass storage Page 0 Inquiry Data */
#[no_mangle]
pub static mut MSC_Page00_Inquiry_Data: [uint8_t; 7] =
    [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t,
     (7 as libc::c_int - 4 as libc::c_int) as uint8_t,
     0 as libc::c_int as uint8_t, 0x80 as libc::c_int as uint8_t,
     0x83 as libc::c_int as uint8_t];
/* USB Mass storage sense 6  Data */
#[no_mangle]
pub static mut MSC_Mode_Sense6_data: [uint8_t; 8] =
    [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
/* USB Mass storage sense 10  Data */
#[no_mangle]
pub static mut MSC_Mode_Sense10_data: [uint8_t; 8] =
    [0 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup MSC_DATA_Private_Functions
  * @{
  */
/* * @defgroup MSC_DATA_Private_FunctionPrototypes
  * @{
  */ 
/* *
  * @}
  */
/* *
  * @}
  */
