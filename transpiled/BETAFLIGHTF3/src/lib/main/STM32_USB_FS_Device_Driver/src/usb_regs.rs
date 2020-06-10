use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type _EP_DBUF_DIR = libc::c_uint;
pub const EP_DBUF_IN: _EP_DBUF_DIR = 2;
pub const EP_DBUF_OUT: _EP_DBUF_DIR = 1;
pub const EP_DBUF_ERR: _EP_DBUF_DIR = 0;
pub type EP_DBUF_DIR = _EP_DBUF_DIR;
/* *
  ******************************************************************************
  * @file    usb_regs.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Interface functions to USB cell registers
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
* Function Name  : SetCNTR.
* Description    : Set the CNTR register value.
* Input          : wRegValue: new register value.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetCNTR(mut wRegValue: uint16_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                wRegValue as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : GetCNTR.
* Description    : returns the CNTR register value.
* Input          : None.
* Output         : None.
* Return         : CNTR register Value.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetCNTR() -> uint16_t {
    return *((0x40005c00 as libc::c_long +
                  0x40 as libc::c_int as libc::c_long) as *mut libc::c_uint)
               as uint16_t;
}
/* ******************************************************************************
* Function Name  : SetISTR.
* Description    : Set the ISTR register value.
* Input          : wRegValue: new register value.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetISTR(mut wRegValue: uint16_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x44 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                wRegValue as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : GetISTR
* Description    : Returns the ISTR register value.
* Input          : None.
* Output         : None.
* Return         : ISTR register Value
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetISTR() -> uint16_t {
    return *((0x40005c00 as libc::c_long +
                  0x44 as libc::c_int as libc::c_long) as *mut libc::c_uint)
               as uint16_t;
}
/* ******************************************************************************
* Function Name  : GetFNR
* Description    : Returns the FNR register value.
* Input          : None.
* Output         : None.
* Return         : FNR register Value
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetFNR() -> uint16_t {
    return *((0x40005c00 as libc::c_long +
                  0x48 as libc::c_int as libc::c_long) as *mut libc::c_uint)
               as uint16_t;
}
/* ******************************************************************************
* Function Name  : SetDADDR
* Description    : Set the DADDR register value.
* Input          : wRegValue: new register value.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetDADDR(mut wRegValue: uint16_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x4c as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                wRegValue as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : GetDADDR
* Description    : Returns the DADDR register value.
* Input          : None.
* Output         : None.
* Return         : DADDR register Value
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetDADDR() -> uint16_t {
    return *((0x40005c00 as libc::c_long +
                  0x4c as libc::c_int as libc::c_long) as *mut libc::c_uint)
               as uint16_t;
}
/* ******************************************************************************
* Function Name  : SetBTABLE
* Description    : Set the BTABLE.
* Input          : wRegValue: New register value.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetBTABLE(mut wRegValue: uint16_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x50 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                (wRegValue as libc::c_int &
                                     0xfff8 as libc::c_int) as uint16_t as
                                    libc::c_uint);
}
/* ******************************************************************************
* Function Name  : GetBTABLE.
* Description    : Returns the BTABLE register value.
* Input          : None. 
* Output         : None.
* Return         : BTABLE address.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetBTABLE() -> uint16_t {
    return (*((0x40005c00 as libc::c_long +
                   0x50 as libc::c_int as libc::c_long) as *mut libc::c_uint)
                as uint16_t as libc::c_int & !(0x7 as libc::c_int)) as
               uint16_t;
}
/* ******************************************************************************
* Function Name  : SetENDPOINT
* Description    : Set the Endpoint register value.
* Input          : bEpNum: Endpoint Number. 
*                  wRegValue.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetENDPOINT(mut bEpNum: uint8_t,
                                     mut wRegValue: uint16_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                wRegValue as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : GetENDPOINT
* Description    : Return the Endpoint register value.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint register value.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetENDPOINT(mut bEpNum: uint8_t) -> uint16_t {
    return *(0x40005c00 as libc::c_long as
                 *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
               uint16_t;
}
/* ******************************************************************************
* Function Name  : SetEPType
* Description    : sets the type in the endpoint register.
* Input          : bEpNum: Endpoint Number. 
*                  wType: type definition.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPType(mut bEpNum: uint8_t, mut wType: uint16_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (*(0x40005c00 as libc::c_long as
                                       *mut libc::c_uint).offset(bEpNum as
                                                                     libc::c_int
                                                                     as isize)
                                     as uint16_t as libc::c_int &
                                     (!(0x600 as libc::c_int) &
                                          (0x8000 as libc::c_int |
                                               0x800 as libc::c_int |
                                               0x600 as libc::c_int |
                                               0x100 as libc::c_int |
                                               0x80 as libc::c_int |
                                               0xf as libc::c_int)) |
                                     wType as libc::c_int) as uint16_t as
                                    libc::c_uint);
}
/* ******************************************************************************
* Function Name  : GetEPType
* Description    : Returns the endpoint type.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint Type
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPType(mut bEpNum: uint8_t) -> uint16_t {
    return (*(0x40005c00 as libc::c_long as
                  *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
                uint16_t as libc::c_int & 0x600 as libc::c_int) as uint16_t;
}
/* ******************************************************************************
* Function Name  : SetEPTxStatus
* Description    : Set the status of Tx endpoint.
* Input          : bEpNum: Endpoint Number. 
*                  wState: new state.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPTxStatus(mut bEpNum: uint8_t,
                                       mut wState: uint16_t) {
    let mut _wRegVal: uint16_t = 0;
    _wRegVal =
        (*(0x40005c00 as libc::c_long as
               *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
             uint16_t as libc::c_int &
             (0x30 as libc::c_int |
                  (0x8000 as libc::c_int | 0x800 as libc::c_int |
                       0x600 as libc::c_int | 0x100 as libc::c_int |
                       0x80 as libc::c_int | 0xf as libc::c_int))) as
            uint16_t;
    if 0x10 as libc::c_int & wState as libc::c_int != 0 as libc::c_int {
        _wRegVal = (_wRegVal as libc::c_int ^ 0x10 as libc::c_int) as uint16_t
    }
    if 0x20 as libc::c_int & wState as libc::c_int != 0 as libc::c_int {
        _wRegVal = (_wRegVal as libc::c_int ^ 0x20 as libc::c_int) as uint16_t
    }
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (_wRegVal as libc::c_int |
                                     0x8000 as libc::c_int |
                                     0x80 as libc::c_int) as uint16_t as
                                    libc::c_uint);
}
/* ******************************************************************************
* Function Name  : SetEPRxStatus
* Description    : Set the status of Rx endpoint.
* Input          : bEpNum: Endpoint Number. 
*                  wState: new state.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPRxStatus(mut bEpNum: uint8_t,
                                       mut wState: uint16_t) {
    let mut _wRegVal: uint16_t = 0;
    _wRegVal =
        (*(0x40005c00 as libc::c_long as
               *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
             uint16_t as libc::c_int &
             (0x3000 as libc::c_int |
                  (0x8000 as libc::c_int | 0x800 as libc::c_int |
                       0x600 as libc::c_int | 0x100 as libc::c_int |
                       0x80 as libc::c_int | 0xf as libc::c_int))) as
            uint16_t;
    if 0x1000 as libc::c_int & wState as libc::c_int != 0 as libc::c_int {
        _wRegVal =
            (_wRegVal as libc::c_int ^ 0x1000 as libc::c_int) as uint16_t
    }
    if 0x2000 as libc::c_int & wState as libc::c_int != 0 as libc::c_int {
        _wRegVal =
            (_wRegVal as libc::c_int ^ 0x2000 as libc::c_int) as uint16_t
    }
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (_wRegVal as libc::c_int |
                                     0x8000 as libc::c_int |
                                     0x80 as libc::c_int) as uint16_t as
                                    libc::c_uint);
}
/* ******************************************************************************
* Function Name  : SetDouBleBuffEPStall
* Description    : sets the status for Double Buffer Endpoint to STALL
* Input          : bEpNum: Endpoint Number. 
*                  bDir: Endpoint direction.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetDouBleBuffEPStall(mut bEpNum: uint8_t,
                                              mut bDir: uint8_t) {
    let mut Endpoint_DTOG_Status: uint16_t = 0;
    Endpoint_DTOG_Status = GetENDPOINT(bEpNum);
    if bDir as libc::c_int == EP_DBUF_OUT as libc::c_int {
        /* OUT double buffered endpoint */
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                         *mut libc::c_uint).offset(bEpNum as
                                                                       libc::c_int
                                                                       as
                                                                       isize),
                                    (Endpoint_DTOG_Status as libc::c_int &
                                         !(0x1000 as libc::c_int)) as
                                        libc::c_uint)
    } else if bDir as libc::c_int == EP_DBUF_IN as libc::c_int {
        /* IN double buffered endpoint */
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                         *mut libc::c_uint).offset(bEpNum as
                                                                       libc::c_int
                                                                       as
                                                                       isize),
                                    (Endpoint_DTOG_Status as libc::c_int &
                                         !(0x10 as libc::c_int)) as
                                        libc::c_uint)
    };
}
/* ******************************************************************************
* Function Name  : GetEPTxStatus
* Description    : Returns the endpoint Tx status.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint TX Status
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPTxStatus(mut bEpNum: uint8_t) -> uint16_t {
    return (*(0x40005c00 as libc::c_long as
                  *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
                uint16_t as libc::c_int & 0x30 as libc::c_int) as uint16_t;
}
/* ******************************************************************************
* Function Name  : GetEPRxStatus
* Description    : Returns the endpoint Rx status.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint RX Status
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPRxStatus(mut bEpNum: uint8_t) -> uint16_t {
    return (*(0x40005c00 as libc::c_long as
                  *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
                uint16_t as libc::c_int & 0x3000 as libc::c_int) as uint16_t;
}
/* ******************************************************************************
* Function Name  : SetEPTxValid
* Description    : Valid the endpoint Tx Status.
* Input          : bEpNum: Endpoint Number.  
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPTxValid(mut bEpNum: uint8_t) {
    let mut _wRegVal: uint16_t = 0;
    _wRegVal =
        (*(0x40005c00 as libc::c_long as
               *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
             uint16_t as libc::c_int &
             (0x30 as libc::c_int |
                  (0x8000 as libc::c_int | 0x800 as libc::c_int |
                       0x600 as libc::c_int | 0x100 as libc::c_int |
                       0x80 as libc::c_int | 0xf as libc::c_int))) as
            uint16_t;
    if 0x10 as libc::c_int & 0x30 as libc::c_int != 0 as libc::c_int {
        _wRegVal = (_wRegVal as libc::c_int ^ 0x10 as libc::c_int) as uint16_t
    }
    if 0x20 as libc::c_int & 0x30 as libc::c_int != 0 as libc::c_int {
        _wRegVal = (_wRegVal as libc::c_int ^ 0x20 as libc::c_int) as uint16_t
    }
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (_wRegVal as libc::c_int |
                                     0x8000 as libc::c_int |
                                     0x80 as libc::c_int) as uint16_t as
                                    libc::c_uint);
}
/* ******************************************************************************
* Function Name  : SetEPRxValid
* Description    : Valid the endpoint Rx Status.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPRxValid(mut bEpNum: uint8_t) {
    let mut _wRegVal: uint16_t = 0;
    _wRegVal =
        (*(0x40005c00 as libc::c_long as
               *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
             uint16_t as libc::c_int &
             (0x3000 as libc::c_int |
                  (0x8000 as libc::c_int | 0x800 as libc::c_int |
                       0x600 as libc::c_int | 0x100 as libc::c_int |
                       0x80 as libc::c_int | 0xf as libc::c_int))) as
            uint16_t;
    if 0x1000 as libc::c_int & 0x3000 as libc::c_int != 0 as libc::c_int {
        _wRegVal =
            (_wRegVal as libc::c_int ^ 0x1000 as libc::c_int) as uint16_t
    }
    if 0x2000 as libc::c_int & 0x3000 as libc::c_int != 0 as libc::c_int {
        _wRegVal =
            (_wRegVal as libc::c_int ^ 0x2000 as libc::c_int) as uint16_t
    }
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (_wRegVal as libc::c_int |
                                     0x8000 as libc::c_int |
                                     0x80 as libc::c_int) as uint16_t as
                                    libc::c_uint);
}
/* ******************************************************************************
* Function Name  : SetEP_KIND
* Description    : Clear the EP_KIND bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEP_KIND(mut bEpNum: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (0x8000 as libc::c_int | 0x80 as libc::c_int |
                                     (*(0x40005c00 as libc::c_long as
                                            *mut libc::c_uint).offset(bEpNum
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          isize)
                                          as uint16_t as libc::c_int |
                                          0x100 as libc::c_int) &
                                         (0x8000 as libc::c_int |
                                              0x800 as libc::c_int |
                                              0x600 as libc::c_int |
                                              0x100 as libc::c_int |
                                              0x80 as libc::c_int |
                                              0xf as libc::c_int)) as uint16_t
                                    as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : ClearEP_KIND
* Description    : set the  EP_KIND bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn ClearEP_KIND(mut bEpNum: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (0x8000 as libc::c_int | 0x80 as libc::c_int |
                                     *(0x40005c00 as libc::c_long as
                                           *mut libc::c_uint).offset(bEpNum as
                                                                         libc::c_int
                                                                         as
                                                                         isize)
                                         as uint16_t as libc::c_int &
                                         (!(0x100 as libc::c_int) &
                                              (0x8000 as libc::c_int |
                                                   0x800 as libc::c_int |
                                                   0x600 as libc::c_int |
                                                   0x100 as libc::c_int |
                                                   0x80 as libc::c_int |
                                                   0xf as libc::c_int))) as
                                    uint16_t as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : Clear_Status_Out
* Description    : Clear the Status Out of the related Endpoint
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Clear_Status_Out(mut bEpNum: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (0x8000 as libc::c_int | 0x80 as libc::c_int |
                                     *(0x40005c00 as libc::c_long as
                                           *mut libc::c_uint).offset(bEpNum as
                                                                         libc::c_int
                                                                         as
                                                                         isize)
                                         as uint16_t as libc::c_int &
                                         (!(0x100 as libc::c_int) &
                                              (0x8000 as libc::c_int |
                                                   0x800 as libc::c_int |
                                                   0x600 as libc::c_int |
                                                   0x100 as libc::c_int |
                                                   0x80 as libc::c_int |
                                                   0xf as libc::c_int))) as
                                    uint16_t as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : Set_Status_Out
* Description    : Set the Status Out of the related Endpoint
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Set_Status_Out(mut bEpNum: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (0x8000 as libc::c_int | 0x80 as libc::c_int |
                                     (*(0x40005c00 as libc::c_long as
                                            *mut libc::c_uint).offset(bEpNum
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          isize)
                                          as uint16_t as libc::c_int |
                                          0x100 as libc::c_int) &
                                         (0x8000 as libc::c_int |
                                              0x800 as libc::c_int |
                                              0x600 as libc::c_int |
                                              0x100 as libc::c_int |
                                              0x80 as libc::c_int |
                                              0xf as libc::c_int)) as uint16_t
                                    as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : SetEPDoubleBuff
* Description    : Enable the double buffer feature for the endpoint. 
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPDoubleBuff(mut bEpNum: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (0x8000 as libc::c_int | 0x80 as libc::c_int |
                                     (*(0x40005c00 as libc::c_long as
                                            *mut libc::c_uint).offset(bEpNum
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          isize)
                                          as uint16_t as libc::c_int |
                                          0x100 as libc::c_int) &
                                         (0x8000 as libc::c_int |
                                              0x800 as libc::c_int |
                                              0x600 as libc::c_int |
                                              0x100 as libc::c_int |
                                              0x80 as libc::c_int |
                                              0xf as libc::c_int)) as uint16_t
                                    as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : ClearEPDoubleBuff
* Description    : Disable the double buffer feature for the endpoint. 
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn ClearEPDoubleBuff(mut bEpNum: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (0x8000 as libc::c_int | 0x80 as libc::c_int |
                                     *(0x40005c00 as libc::c_long as
                                           *mut libc::c_uint).offset(bEpNum as
                                                                         libc::c_int
                                                                         as
                                                                         isize)
                                         as uint16_t as libc::c_int &
                                         (!(0x100 as libc::c_int) &
                                              (0x8000 as libc::c_int |
                                                   0x800 as libc::c_int |
                                                   0x600 as libc::c_int |
                                                   0x100 as libc::c_int |
                                                   0x80 as libc::c_int |
                                                   0xf as libc::c_int))) as
                                    uint16_t as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : GetTxStallStatus
* Description    : Returns the Stall status of the Tx endpoint.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Tx Stall status.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetTxStallStatus(mut bEpNum: uint8_t) -> uint16_t {
    return (*(0x40005c00 as libc::c_long as
                  *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
                uint16_t as libc::c_int & 0x30 as libc::c_int ==
                0x10 as libc::c_int) as libc::c_int as uint16_t;
}
/* ******************************************************************************
* Function Name  : GetRxStallStatus
* Description    : Returns the Stall status of the Rx endpoint. 
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx Stall status.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetRxStallStatus(mut bEpNum: uint8_t) -> uint16_t {
    return (*(0x40005c00 as libc::c_long as
                  *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
                uint16_t as libc::c_int & 0x3000 as libc::c_int ==
                0x1000 as libc::c_int) as libc::c_int as uint16_t;
}
/* ******************************************************************************
* Function Name  : ClearEP_CTR_RX
* Description    : Clear the CTR_RX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn ClearEP_CTR_RX(mut bEpNum: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (*(0x40005c00 as libc::c_long as
                                       *mut libc::c_uint).offset(bEpNum as
                                                                     libc::c_int
                                                                     as isize)
                                     as uint16_t as libc::c_int &
                                     0x7fff as libc::c_int &
                                     (0x8000 as libc::c_int |
                                          0x800 as libc::c_int |
                                          0x600 as libc::c_int |
                                          0x100 as libc::c_int |
                                          0x80 as libc::c_int |
                                          0xf as libc::c_int)) as
                                    libc::c_uint);
}
/* ******************************************************************************
* Function Name  : ClearEP_CTR_TX
* Description    : Clear the CTR_TX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn ClearEP_CTR_TX(mut bEpNum: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (*(0x40005c00 as libc::c_long as
                                       *mut libc::c_uint).offset(bEpNum as
                                                                     libc::c_int
                                                                     as isize)
                                     as uint16_t as libc::c_int &
                                     0xff7f as libc::c_int &
                                     (0x8000 as libc::c_int |
                                          0x800 as libc::c_int |
                                          0x600 as libc::c_int |
                                          0x100 as libc::c_int |
                                          0x80 as libc::c_int |
                                          0xf as libc::c_int)) as
                                    libc::c_uint);
}
/* ******************************************************************************
* Function Name  : ToggleDTOG_RX
* Description    : Toggle the DTOG_RX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn ToggleDTOG_RX(mut bEpNum: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (0x8000 as libc::c_int as uint16_t as
                                     libc::c_int | 0x80 as libc::c_int |
                                     0x4000 as libc::c_int |
                                     *(0x40005c00 as libc::c_long as
                                           *mut libc::c_uint).offset(bEpNum as
                                                                         libc::c_int
                                                                         as
                                                                         isize)
                                         as uint16_t as libc::c_int &
                                         (0x8000 as libc::c_int |
                                              0x800 as libc::c_int |
                                              0x600 as libc::c_int |
                                              0x100 as libc::c_int |
                                              0x80 as libc::c_int |
                                              0xf as libc::c_int)) as
                                    libc::c_uint);
}
/* ******************************************************************************
* Function Name  : ToggleDTOG_TX
* Description    : Toggle the DTOG_TX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn ToggleDTOG_TX(mut bEpNum: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (0x8000 as libc::c_int as uint16_t as
                                     libc::c_int | 0x80 as libc::c_int |
                                     0x40 as libc::c_int |
                                     *(0x40005c00 as libc::c_long as
                                           *mut libc::c_uint).offset(bEpNum as
                                                                         libc::c_int
                                                                         as
                                                                         isize)
                                         as uint16_t as libc::c_int &
                                         (0x8000 as libc::c_int |
                                              0x800 as libc::c_int |
                                              0x600 as libc::c_int |
                                              0x100 as libc::c_int |
                                              0x80 as libc::c_int |
                                              0xf as libc::c_int)) as
                                    libc::c_uint);
}
/* ******************************************************************************
* Function Name  : ClearDTOG_RX.
* Description    : Clear the DTOG_RX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn ClearDTOG_RX(mut bEpNum: uint8_t) {
    if *(0x40005c00 as libc::c_long as
             *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
           uint16_t as libc::c_int & 0x4000 as libc::c_int != 0 as libc::c_int
       {
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                         *mut libc::c_uint).offset(bEpNum as
                                                                       libc::c_int
                                                                       as
                                                                       isize),
                                    (0x8000 as libc::c_int as uint16_t as
                                         libc::c_int | 0x80 as libc::c_int |
                                         0x4000 as libc::c_int |
                                         *(0x40005c00 as libc::c_long as
                                               *mut libc::c_uint).offset(bEpNum
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             isize)
                                             as uint16_t as libc::c_int &
                                             (0x8000 as libc::c_int |
                                                  0x800 as libc::c_int |
                                                  0x600 as libc::c_int |
                                                  0x100 as libc::c_int |
                                                  0x80 as libc::c_int |
                                                  0xf as libc::c_int)) as
                                        libc::c_uint)
    };
}
/* ******************************************************************************
* Function Name  : ClearDTOG_TX.
* Description    : Clear the DTOG_TX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn ClearDTOG_TX(mut bEpNum: uint8_t) {
    if *(0x40005c00 as libc::c_long as
             *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
           uint16_t as libc::c_int & 0x40 as libc::c_int != 0 as libc::c_int {
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                         *mut libc::c_uint).offset(bEpNum as
                                                                       libc::c_int
                                                                       as
                                                                       isize),
                                    (0x8000 as libc::c_int as uint16_t as
                                         libc::c_int | 0x80 as libc::c_int |
                                         0x40 as libc::c_int |
                                         *(0x40005c00 as libc::c_long as
                                               *mut libc::c_uint).offset(bEpNum
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             isize)
                                             as uint16_t as libc::c_int &
                                             (0x8000 as libc::c_int |
                                                  0x800 as libc::c_int |
                                                  0x600 as libc::c_int |
                                                  0x100 as libc::c_int |
                                                  0x80 as libc::c_int |
                                                  0xf as libc::c_int)) as
                                        libc::c_uint)
    };
}
/* ******************************************************************************
* Function Name  : SetEPAddress
* Description    : Set the endpoint address.
* Input          : bEpNum: Endpoint Number.
*                  bAddr: New endpoint address.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPAddress(mut bEpNum: uint8_t,
                                      mut bAddr: uint8_t) {
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                     *mut libc::c_uint).offset(bEpNum as
                                                                   libc::c_int
                                                                   as isize),
                                (0x8000 as libc::c_int as uint16_t as
                                     libc::c_int | 0x80 as libc::c_int |
                                     *(0x40005c00 as libc::c_long as
                                           *mut libc::c_uint).offset(bEpNum as
                                                                         libc::c_int
                                                                         as
                                                                         isize)
                                         as uint16_t as libc::c_int &
                                         (0x8000 as libc::c_int |
                                              0x800 as libc::c_int |
                                              0x600 as libc::c_int |
                                              0x100 as libc::c_int |
                                              0x80 as libc::c_int |
                                              0xf as libc::c_int) |
                                     bAddr as libc::c_int) as libc::c_uint);
}
/* ******************************************************************************
* Function Name  : GetEPAddress
* Description    : Get the endpoint address.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint address.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPAddress(mut bEpNum: uint8_t) -> uint8_t {
    return (*(0x40005c00 as libc::c_long as
                  *mut libc::c_uint).offset(bEpNum as libc::c_int as isize) as
                uint16_t as libc::c_int & 0xf as libc::c_int) as uint8_t;
}
/* ******************************************************************************
* Function Name  : SetEPTxAddr
* Description    : Set the endpoint Tx buffer address.
* Input          : bEpNum: Endpoint Number.
*                  wAddr: new address. 
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPTxAddr(mut bEpNum: uint8_t,
                                     mut wAddr: uint16_t) {
    ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                           0x50 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t as
                                        libc::c_int & !(0x7 as libc::c_int)) +
                                       bEpNum as libc::c_int *
                                           8 as libc::c_int) *
                                      2 as libc::c_int) as libc::c_long +
                                     0x40006000 as libc::c_long) as
                                    *mut uint32_t,
                                ((wAddr as libc::c_int >> 1 as libc::c_int) <<
                                     1 as libc::c_int) as uint32_t);
}
/* ******************************************************************************
* Function Name  : SetEPRxAddr
* Description    : Set the endpoint Rx buffer address.
* Input          : bEpNum: Endpoint Number.
*                  wAddr: new address.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPRxAddr(mut bEpNum: uint8_t,
                                     mut wAddr: uint16_t) {
    ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                           0x50 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t as
                                        libc::c_int & !(0x7 as libc::c_int)) +
                                       bEpNum as libc::c_int *
                                           8 as libc::c_int +
                                       4 as libc::c_int) * 2 as libc::c_int)
                                     as libc::c_long +
                                     0x40006000 as libc::c_long) as
                                    *mut uint32_t,
                                ((wAddr as libc::c_int >> 1 as libc::c_int) <<
                                     1 as libc::c_int) as uint32_t);
}
/* ******************************************************************************
* Function Name  : GetEPTxAddr
* Description    : Returns the endpoint Tx buffer address.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx buffer address. 
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPTxAddr(mut bEpNum: uint8_t) -> uint16_t {
    return *(((((*((0x40005c00 as libc::c_long +
                        0x50 as libc::c_int as libc::c_long) as
                       *mut libc::c_uint) as uint16_t as libc::c_int &
                     !(0x7 as libc::c_int)) +
                    bEpNum as libc::c_int * 8 as libc::c_int) *
                   2 as libc::c_int) as libc::c_long +
                  0x40006000 as libc::c_long) as *mut uint32_t) as uint16_t;
}
/* ******************************************************************************
* Function Name  : GetEPRxAddr.
* Description    : Returns the endpoint Rx buffer address.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx buffer address.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPRxAddr(mut bEpNum: uint8_t) -> uint16_t {
    return *(((((*((0x40005c00 as libc::c_long +
                        0x50 as libc::c_int as libc::c_long) as
                       *mut libc::c_uint) as uint16_t as libc::c_int &
                     !(0x7 as libc::c_int)) +
                    bEpNum as libc::c_int * 8 as libc::c_int +
                    4 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                  0x40006000 as libc::c_long) as *mut uint32_t) as uint16_t;
}
/* ******************************************************************************
* Function Name  : SetEPTxCount.
* Description    : Set the Tx count.
* Input          : bEpNum: Endpoint Number.
*                  wCount: new count value.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPTxCount(mut bEpNum: uint8_t,
                                      mut wCount: uint16_t) {
    ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                           0x50 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t as
                                        libc::c_int & !(0x7 as libc::c_int)) +
                                       bEpNum as libc::c_int *
                                           8 as libc::c_int +
                                       2 as libc::c_int) * 2 as libc::c_int)
                                     as libc::c_long +
                                     0x40006000 as libc::c_long) as
                                    *mut uint32_t, wCount as uint32_t);
}
/* ******************************************************************************
* Function Name  : SetEPCountRxReg.
* Description    : Set the Count Rx Register value.
* Input          : *pdwReg: point to the register.
*                  wCount: the new register value.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPCountRxReg(mut pdwReg: *mut uint32_t,
                                         mut wCount: uint16_t) {
    let mut wNBlocks: uint16_t = 0;
    if wCount as libc::c_int > 62 as libc::c_int {
        wNBlocks = (wCount as libc::c_int >> 5 as libc::c_int) as uint16_t;
        if wCount as libc::c_int & 0x1f as libc::c_int == 0 as libc::c_int {
            wNBlocks = wNBlocks.wrapping_sub(1)
        }
        *pdwReg =
            ((wNBlocks as libc::c_int) << 10 as libc::c_int |
                 0x8000 as libc::c_int) as uint32_t
    } else {
        wNBlocks = (wCount as libc::c_int >> 1 as libc::c_int) as uint16_t;
        if wCount as libc::c_int & 0x1 as libc::c_int != 0 as libc::c_int {
            wNBlocks = wNBlocks.wrapping_add(1)
        }
        *pdwReg = ((wNBlocks as libc::c_int) << 10 as libc::c_int) as uint32_t
    };
}
/* ******************************************************************************
* Function Name  : SetEPRxCount
* Description    : Set the Rx count.
* Input          : bEpNum: Endpoint Number. 
*                  wCount: the new count value.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPRxCount(mut bEpNum: uint8_t,
                                      mut wCount: uint16_t) {
    let mut pdwReg: *mut uint32_t =
        ((((*((0x40005c00 as libc::c_long +
                   0x50 as libc::c_int as libc::c_long) as *mut libc::c_uint)
                as uint16_t as libc::c_int & !(0x7 as libc::c_int)) +
               bEpNum as libc::c_int * 8 as libc::c_int + 6 as libc::c_int) *
              2 as libc::c_int) as libc::c_long + 0x40006000 as libc::c_long)
            as *mut uint32_t;
    let mut wNBlocks: uint16_t = 0;
    if wCount as libc::c_int > 62 as libc::c_int {
        wNBlocks = (wCount as libc::c_int >> 5 as libc::c_int) as uint16_t;
        if wCount as libc::c_int & 0x1f as libc::c_int == 0 as libc::c_int {
            wNBlocks = wNBlocks.wrapping_sub(1)
        }
        ::core::ptr::write_volatile(pdwReg,
                                    ((wNBlocks as libc::c_int) <<
                                         10 as libc::c_int |
                                         0x8000 as libc::c_int) as uint32_t)
    } else {
        wNBlocks = (wCount as libc::c_int >> 1 as libc::c_int) as uint16_t;
        if wCount as libc::c_int & 0x1 as libc::c_int != 0 as libc::c_int {
            wNBlocks = wNBlocks.wrapping_add(1)
        }
        ::core::ptr::write_volatile(pdwReg,
                                    ((wNBlocks as libc::c_int) <<
                                         10 as libc::c_int) as uint32_t)
    };
}
/* ******************************************************************************
* Function Name  : GetEPTxCount
* Description    : Get the Tx count.
* Input          : bEpNum: Endpoint Number. 
* Output         : None
* Return         : Tx count value.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPTxCount(mut bEpNum: uint8_t) -> uint16_t {
    return (*(((((*((0x40005c00 as libc::c_long +
                         0x50 as libc::c_int as libc::c_long) as
                        *mut libc::c_uint) as uint16_t as libc::c_int &
                      !(0x7 as libc::c_int)) +
                     bEpNum as libc::c_int * 8 as libc::c_int +
                     2 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                   0x40006000 as libc::c_long) as *mut uint32_t) as uint16_t
                as libc::c_int & 0x3ff as libc::c_int) as uint16_t;
}
/* ******************************************************************************
* Function Name  : GetEPRxCount
* Description    : Get the Rx count.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx count value.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPRxCount(mut bEpNum: uint8_t) -> uint16_t {
    return (*(((((*((0x40005c00 as libc::c_long +
                         0x50 as libc::c_int as libc::c_long) as
                        *mut libc::c_uint) as uint16_t as libc::c_int &
                      !(0x7 as libc::c_int)) +
                     bEpNum as libc::c_int * 8 as libc::c_int +
                     6 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                   0x40006000 as libc::c_long) as *mut uint32_t) as uint16_t
                as libc::c_int & 0x3ff as libc::c_int) as uint16_t;
}
/* ******************************************************************************
* Function Name  : SetEPDblBuffAddr
* Description    : Set the addresses of the buffer 0 and 1.
* Input          : bEpNum: Endpoint Number.  
*                  wBuf0Addr: new address of buffer 0. 
*                  wBuf1Addr: new address of buffer 1.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPDblBuffAddr(mut bEpNum: uint8_t,
                                          mut wBuf0Addr: uint16_t,
                                          mut wBuf1Addr: uint16_t) {
    ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                           0x50 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t as
                                        libc::c_int & !(0x7 as libc::c_int)) +
                                       bEpNum as libc::c_int *
                                           8 as libc::c_int) *
                                      2 as libc::c_int) as libc::c_long +
                                     0x40006000 as libc::c_long) as
                                    *mut uint32_t,
                                ((wBuf0Addr as libc::c_int >>
                                      1 as libc::c_int) << 1 as libc::c_int)
                                    as uint32_t);
    ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                           0x50 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t as
                                        libc::c_int & !(0x7 as libc::c_int)) +
                                       bEpNum as libc::c_int *
                                           8 as libc::c_int +
                                       4 as libc::c_int) * 2 as libc::c_int)
                                     as libc::c_long +
                                     0x40006000 as libc::c_long) as
                                    *mut uint32_t,
                                ((wBuf1Addr as libc::c_int >>
                                      1 as libc::c_int) << 1 as libc::c_int)
                                    as uint32_t);
}
/* ******************************************************************************
* Function Name  : SetEPDblBuf0Addr
* Description    : Set the Buffer 1 address.
* Input          : bEpNum: Endpoint Number
*                  wBuf0Addr: new address.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPDblBuf0Addr(mut bEpNum: uint8_t,
                                          mut wBuf0Addr: uint16_t) {
    ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                           0x50 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t as
                                        libc::c_int & !(0x7 as libc::c_int)) +
                                       bEpNum as libc::c_int *
                                           8 as libc::c_int) *
                                      2 as libc::c_int) as libc::c_long +
                                     0x40006000 as libc::c_long) as
                                    *mut uint32_t,
                                ((wBuf0Addr as libc::c_int >>
                                      1 as libc::c_int) << 1 as libc::c_int)
                                    as uint32_t);
}
/* ******************************************************************************
* Function Name  : SetEPDblBuf1Addr
* Description    : Set the Buffer 1 address.
* Input          : bEpNum: Endpoint Number
*                  wBuf1Addr: new address.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPDblBuf1Addr(mut bEpNum: uint8_t,
                                          mut wBuf1Addr: uint16_t) {
    ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                           0x50 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t as
                                        libc::c_int & !(0x7 as libc::c_int)) +
                                       bEpNum as libc::c_int *
                                           8 as libc::c_int +
                                       4 as libc::c_int) * 2 as libc::c_int)
                                     as libc::c_long +
                                     0x40006000 as libc::c_long) as
                                    *mut uint32_t,
                                ((wBuf1Addr as libc::c_int >>
                                      1 as libc::c_int) << 1 as libc::c_int)
                                    as uint32_t);
}
/* ******************************************************************************
* Function Name  : GetEPDblBuf0Addr
* Description    : Returns the address of the Buffer 0.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPDblBuf0Addr(mut bEpNum: uint8_t) -> uint16_t {
    return *(((((*((0x40005c00 as libc::c_long +
                        0x50 as libc::c_int as libc::c_long) as
                       *mut libc::c_uint) as uint16_t as libc::c_int &
                     !(0x7 as libc::c_int)) +
                    bEpNum as libc::c_int * 8 as libc::c_int) *
                   2 as libc::c_int) as libc::c_long +
                  0x40006000 as libc::c_long) as *mut uint32_t) as uint16_t;
}
/* ******************************************************************************
* Function Name  : GetEPDblBuf1Addr
* Description    : Returns the address of the Buffer 1.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : Address of the Buffer 1.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPDblBuf1Addr(mut bEpNum: uint8_t) -> uint16_t {
    return *(((((*((0x40005c00 as libc::c_long +
                        0x50 as libc::c_int as libc::c_long) as
                       *mut libc::c_uint) as uint16_t as libc::c_int &
                     !(0x7 as libc::c_int)) +
                    bEpNum as libc::c_int * 8 as libc::c_int +
                    4 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                  0x40006000 as libc::c_long) as *mut uint32_t) as uint16_t;
}
/* ******************************************************************************
* Function Name  : SetEPDblBuffCount
* Description    : Set the number of bytes for a double Buffer 
*                  endpoint.
* Input          : bEpNum,bDir, wCount
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPDblBuffCount(mut bEpNum: uint8_t,
                                           mut bDir: uint8_t,
                                           mut wCount: uint16_t) {
    if bDir as libc::c_int == EP_DBUF_OUT as libc::c_int {
        let mut pdwReg: *mut uint32_t =
            ((((*((0x40005c00 as libc::c_long +
                       0x50 as libc::c_int as libc::c_long) as
                      *mut libc::c_uint) as uint16_t as libc::c_int &
                    !(0x7 as libc::c_int)) +
                   bEpNum as libc::c_int * 8 as libc::c_int +
                   2 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                 0x40006000 as libc::c_long) as *mut uint32_t;
        let mut wNBlocks: uint16_t = 0;
        if wCount as libc::c_int > 62 as libc::c_int {
            wNBlocks =
                (wCount as libc::c_int >> 5 as libc::c_int) as uint16_t;
            if wCount as libc::c_int & 0x1f as libc::c_int == 0 as libc::c_int
               {
                wNBlocks = wNBlocks.wrapping_sub(1)
            }
            ::core::ptr::write_volatile(pdwReg,
                                        ((wNBlocks as libc::c_int) <<
                                             10 as libc::c_int |
                                             0x8000 as libc::c_int) as
                                            uint32_t)
        } else {
            wNBlocks =
                (wCount as libc::c_int >> 1 as libc::c_int) as uint16_t;
            if wCount as libc::c_int & 0x1 as libc::c_int != 0 as libc::c_int
               {
                wNBlocks = wNBlocks.wrapping_add(1)
            }
            ::core::ptr::write_volatile(pdwReg,
                                        ((wNBlocks as libc::c_int) <<
                                             10 as libc::c_int) as uint32_t)
        }
    } else if bDir as libc::c_int == EP_DBUF_IN as libc::c_int {
        ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                               0x50 as libc::c_int as
                                                   libc::c_long) as
                                              *mut libc::c_uint) as uint16_t
                                            as libc::c_int &
                                            !(0x7 as libc::c_int)) +
                                           bEpNum as libc::c_int *
                                               8 as libc::c_int +
                                           2 as libc::c_int) *
                                          2 as libc::c_int) as libc::c_long +
                                         0x40006000 as libc::c_long) as
                                        *mut uint32_t, wCount as uint32_t)
    }
    if bDir as libc::c_int == EP_DBUF_OUT as libc::c_int {
        let mut pdwReg_0: *mut uint32_t =
            ((((*((0x40005c00 as libc::c_long +
                       0x50 as libc::c_int as libc::c_long) as
                      *mut libc::c_uint) as uint16_t as libc::c_int &
                    !(0x7 as libc::c_int)) +
                   bEpNum as libc::c_int * 8 as libc::c_int +
                   6 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                 0x40006000 as libc::c_long) as *mut uint32_t;
        let mut wNBlocks_0: uint16_t = 0;
        if wCount as libc::c_int > 62 as libc::c_int {
            wNBlocks_0 =
                (wCount as libc::c_int >> 5 as libc::c_int) as uint16_t;
            if wCount as libc::c_int & 0x1f as libc::c_int == 0 as libc::c_int
               {
                wNBlocks_0 = wNBlocks_0.wrapping_sub(1)
            }
            ::core::ptr::write_volatile(pdwReg_0,
                                        ((wNBlocks_0 as libc::c_int) <<
                                             10 as libc::c_int |
                                             0x8000 as libc::c_int) as
                                            uint32_t)
        } else {
            wNBlocks_0 =
                (wCount as libc::c_int >> 1 as libc::c_int) as uint16_t;
            if wCount as libc::c_int & 0x1 as libc::c_int != 0 as libc::c_int
               {
                wNBlocks_0 = wNBlocks_0.wrapping_add(1)
            }
            ::core::ptr::write_volatile(pdwReg_0,
                                        ((wNBlocks_0 as libc::c_int) <<
                                             10 as libc::c_int) as uint32_t)
        }
    } else if bDir as libc::c_int == EP_DBUF_IN as libc::c_int {
        ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                               0x50 as libc::c_int as
                                                   libc::c_long) as
                                              *mut libc::c_uint) as uint16_t
                                            as libc::c_int &
                                            !(0x7 as libc::c_int)) +
                                           bEpNum as libc::c_int *
                                               8 as libc::c_int +
                                           6 as libc::c_int) *
                                          2 as libc::c_int) as libc::c_long +
                                         0x40006000 as libc::c_long) as
                                        *mut uint32_t, wCount as uint32_t)
    };
}
/* ******************************************************************************
* Function Name  : SetEPDblBuf0Count
* Description    : Set the number of bytes in the buffer 0 of a double Buffer 
*                  endpoint.
* Input          : bEpNum, bDir,  wCount
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPDblBuf0Count(mut bEpNum: uint8_t,
                                           mut bDir: uint8_t,
                                           mut wCount: uint16_t) {
    if bDir as libc::c_int == EP_DBUF_OUT as libc::c_int {
        let mut pdwReg: *mut uint32_t =
            ((((*((0x40005c00 as libc::c_long +
                       0x50 as libc::c_int as libc::c_long) as
                      *mut libc::c_uint) as uint16_t as libc::c_int &
                    !(0x7 as libc::c_int)) +
                   bEpNum as libc::c_int * 8 as libc::c_int +
                   2 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                 0x40006000 as libc::c_long) as *mut uint32_t;
        let mut wNBlocks: uint16_t = 0;
        if wCount as libc::c_int > 62 as libc::c_int {
            wNBlocks =
                (wCount as libc::c_int >> 5 as libc::c_int) as uint16_t;
            if wCount as libc::c_int & 0x1f as libc::c_int == 0 as libc::c_int
               {
                wNBlocks = wNBlocks.wrapping_sub(1)
            }
            ::core::ptr::write_volatile(pdwReg,
                                        ((wNBlocks as libc::c_int) <<
                                             10 as libc::c_int |
                                             0x8000 as libc::c_int) as
                                            uint32_t)
        } else {
            wNBlocks =
                (wCount as libc::c_int >> 1 as libc::c_int) as uint16_t;
            if wCount as libc::c_int & 0x1 as libc::c_int != 0 as libc::c_int
               {
                wNBlocks = wNBlocks.wrapping_add(1)
            }
            ::core::ptr::write_volatile(pdwReg,
                                        ((wNBlocks as libc::c_int) <<
                                             10 as libc::c_int) as uint32_t)
        }
    } else if bDir as libc::c_int == EP_DBUF_IN as libc::c_int {
        ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                               0x50 as libc::c_int as
                                                   libc::c_long) as
                                              *mut libc::c_uint) as uint16_t
                                            as libc::c_int &
                                            !(0x7 as libc::c_int)) +
                                           bEpNum as libc::c_int *
                                               8 as libc::c_int +
                                           2 as libc::c_int) *
                                          2 as libc::c_int) as libc::c_long +
                                         0x40006000 as libc::c_long) as
                                        *mut uint32_t, wCount as uint32_t)
    };
}
/* ******************************************************************************
* Function Name  : SetEPDblBuf1Count
* Description    : Set the number of bytes in the buffer 0 of a double Buffer 
*                  endpoint.
* Input          : bEpNum,  bDir,  wCount
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn SetEPDblBuf1Count(mut bEpNum: uint8_t,
                                           mut bDir: uint8_t,
                                           mut wCount: uint16_t) {
    if bDir as libc::c_int == EP_DBUF_OUT as libc::c_int {
        let mut pdwReg: *mut uint32_t =
            ((((*((0x40005c00 as libc::c_long +
                       0x50 as libc::c_int as libc::c_long) as
                      *mut libc::c_uint) as uint16_t as libc::c_int &
                    !(0x7 as libc::c_int)) +
                   bEpNum as libc::c_int * 8 as libc::c_int +
                   6 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                 0x40006000 as libc::c_long) as *mut uint32_t;
        let mut wNBlocks: uint16_t = 0;
        if wCount as libc::c_int > 62 as libc::c_int {
            wNBlocks =
                (wCount as libc::c_int >> 5 as libc::c_int) as uint16_t;
            if wCount as libc::c_int & 0x1f as libc::c_int == 0 as libc::c_int
               {
                wNBlocks = wNBlocks.wrapping_sub(1)
            }
            ::core::ptr::write_volatile(pdwReg,
                                        ((wNBlocks as libc::c_int) <<
                                             10 as libc::c_int |
                                             0x8000 as libc::c_int) as
                                            uint32_t)
        } else {
            wNBlocks =
                (wCount as libc::c_int >> 1 as libc::c_int) as uint16_t;
            if wCount as libc::c_int & 0x1 as libc::c_int != 0 as libc::c_int
               {
                wNBlocks = wNBlocks.wrapping_add(1)
            }
            ::core::ptr::write_volatile(pdwReg,
                                        ((wNBlocks as libc::c_int) <<
                                             10 as libc::c_int) as uint32_t)
        }
    } else if bDir as libc::c_int == EP_DBUF_IN as libc::c_int {
        ::core::ptr::write_volatile(((((*((0x40005c00 as libc::c_long +
                                               0x50 as libc::c_int as
                                                   libc::c_long) as
                                              *mut libc::c_uint) as uint16_t
                                            as libc::c_int &
                                            !(0x7 as libc::c_int)) +
                                           bEpNum as libc::c_int *
                                               8 as libc::c_int +
                                           6 as libc::c_int) *
                                          2 as libc::c_int) as libc::c_long +
                                         0x40006000 as libc::c_long) as
                                        *mut uint32_t, wCount as uint32_t)
    };
}
/* ******************************************************************************
* Function Name  : GetEPDblBuf0Count
* Description    : Returns the number of byte received in the buffer 0 of a double
*                  Buffer endpoint.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : Endpoint Buffer 0 count
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPDblBuf0Count(mut bEpNum: uint8_t) -> uint16_t {
    return (*(((((*((0x40005c00 as libc::c_long +
                         0x50 as libc::c_int as libc::c_long) as
                        *mut libc::c_uint) as uint16_t as libc::c_int &
                      !(0x7 as libc::c_int)) +
                     bEpNum as libc::c_int * 8 as libc::c_int +
                     2 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                   0x40006000 as libc::c_long) as *mut uint32_t) as uint16_t
                as libc::c_int & 0x3ff as libc::c_int) as uint16_t;
}
/* ******************************************************************************
* Function Name  : GetEPDblBuf1Count
* Description    : Returns the number of data received in the buffer 1 of a double
*                  Buffer endpoint.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : Endpoint Buffer 1 count.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPDblBuf1Count(mut bEpNum: uint8_t) -> uint16_t {
    return (*(((((*((0x40005c00 as libc::c_long +
                         0x50 as libc::c_int as libc::c_long) as
                        *mut libc::c_uint) as uint16_t as libc::c_int &
                      !(0x7 as libc::c_int)) +
                     bEpNum as libc::c_int * 8 as libc::c_int +
                     6 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                   0x40006000 as libc::c_long) as *mut uint32_t) as uint16_t
                as libc::c_int & 0x3ff as libc::c_int) as uint16_t;
}
/* ******************************************************************************
* Function Name  : GetEPDblBufDir
* Description    : gets direction of the double buffered endpoint
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : EP_DBUF_OUT, EP_DBUF_IN,
*                  EP_DBUF_ERR if the endpoint counter not yet programmed.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn GetEPDblBufDir(mut bEpNum: uint8_t) -> EP_DBUF_DIR {
    if (*(((((*((0x40005c00 as libc::c_long +
                     0x50 as libc::c_int as libc::c_long) as
                    *mut libc::c_uint) as uint16_t as libc::c_int &
                  !(0x7 as libc::c_int)) +
                 bEpNum as libc::c_int * 8 as libc::c_int + 6 as libc::c_int)
                * 2 as libc::c_int) as libc::c_long +
               0x40006000 as libc::c_long) as *mut uint32_t) &
            0xfc00 as libc::c_int as libc::c_uint) as uint16_t as libc::c_int
           != 0 as libc::c_int {
        return EP_DBUF_OUT
    } else if *(((((*((0x40005c00 as libc::c_long +
                           0x50 as libc::c_int as libc::c_long) as
                          *mut libc::c_uint) as uint16_t as libc::c_int &
                        !(0x7 as libc::c_int)) +
                       bEpNum as libc::c_int * 8 as libc::c_int +
                       2 as libc::c_int) * 2 as libc::c_int) as libc::c_long +
                     0x40006000 as libc::c_long) as *mut uint32_t) as uint16_t
                  as libc::c_int & 0x3ff as libc::c_int != 0 as libc::c_int {
        return EP_DBUF_IN
    } else { return EP_DBUF_ERR };
}
/* ******************************************************************************
* Function Name  : FreeUserBuffer
* Description    : free buffer used from the application realizing it to the line
                   toggles bit SW_BUF in the double buffered endpoint register
* Input          : bEpNum, bDir
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn FreeUserBuffer(mut bEpNum: uint8_t,
                                        mut bDir: uint8_t) {
    if bDir as libc::c_int == EP_DBUF_OUT as libc::c_int {
        /* OUT double buffered endpoint */
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                         *mut libc::c_uint).offset(bEpNum as
                                                                       libc::c_int
                                                                       as
                                                                       isize),
                                    (0x8000 as libc::c_int as uint16_t as
                                         libc::c_int | 0x80 as libc::c_int |
                                         0x40 as libc::c_int |
                                         *(0x40005c00 as libc::c_long as
                                               *mut libc::c_uint).offset(bEpNum
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             isize)
                                             as uint16_t as libc::c_int &
                                             (0x8000 as libc::c_int |
                                                  0x800 as libc::c_int |
                                                  0x600 as libc::c_int |
                                                  0x100 as libc::c_int |
                                                  0x80 as libc::c_int |
                                                  0xf as libc::c_int)) as
                                        libc::c_uint)
    } else if bDir as libc::c_int == EP_DBUF_IN as libc::c_int {
        /* IN double buffered endpoint */
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                         *mut libc::c_uint).offset(bEpNum as
                                                                       libc::c_int
                                                                       as
                                                                       isize),
                                    (0x8000 as libc::c_int as uint16_t as
                                         libc::c_int | 0x80 as libc::c_int |
                                         0x4000 as libc::c_int |
                                         *(0x40005c00 as libc::c_long as
                                               *mut libc::c_uint).offset(bEpNum
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             isize)
                                             as uint16_t as libc::c_int &
                                             (0x8000 as libc::c_int |
                                                  0x800 as libc::c_int |
                                                  0x600 as libc::c_int |
                                                  0x100 as libc::c_int |
                                                  0x80 as libc::c_int |
                                                  0xf as libc::c_int)) as
                                        libc::c_uint)
    };
}
/* ******************************************************************************
* Function Name  : ToWord
* Description    : merge two byte in a word.
* Input          : bh: byte high, bl: bytes low.
* Output         : None.
* Return         : resulted word.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn ToWord(mut bh: uint8_t, mut bl: uint8_t)
 -> uint16_t {
    let mut wRet: uint16_t = 0;
    wRet =
        (bl as uint16_t as libc::c_int |
             (bh as uint16_t as libc::c_int) << 8 as libc::c_int) as uint16_t;
    return wRet;
}
/* *
  ******************************************************************************
  * @file    usb_regs.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Interface prototype functions to USB cell registers
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
/* double buffered endpoint direction */
/* endpoint buffer number */
/* Exported constants --------------------------------------------------------*/
/* USB_IP Peripheral Registers base address */
/* USB_IP Packet Memory Area base address   */
/* *****************************************************************************/
/*                         General registers                                  */
/* *****************************************************************************/
/* Control register */
/* Interrupt status register */
/* Frame number register */
/* Device address register */
/* Buffer Table address register */
/* *****************************************************************************/
/*                         Endpoint registers                                 */
/* *****************************************************************************/
/* endpoint 0 register address */
/* Endpoint Addresses (w/direction) */
/* endpoints enumeration */
/* *****************************************************************************/
/*                       ISTR interrupt events                                */
/* *****************************************************************************/
/* Correct TRansfer (clear-only bit) */
/* DMA OVeR/underrun (clear-only bit) */
/* ERRor (clear-only bit) */
/* WaKe UP (clear-only bit) */
/* SUSPend (clear-only bit) */
/* RESET (clear-only bit) */
/* Start Of Frame (clear-only bit) */
/* Expected Start Of Frame (clear-only bit) */
/* DIRection of transaction (read-only bit)  */
/* EndPoint IDentifier (read-only bit)  */
/* clear Correct TRansfer bit */
/* clear DMA OVeR/underrun bit*/
/* clear ERRor bit */
/* clear WaKe UP bit     */
/* clear SUSPend bit     */
/* clear RESET bit      */
/* clear Start Of Frame bit   */
/* clear Expected Start Of Frame bit */
/* *****************************************************************************/
/*             CNTR control register bits definitions                         */
/* *****************************************************************************/
/* Correct TRansfer Mask */
/* DMA OVeR/underrun Mask */
/* ERRor Mask */
/* WaKe UP Mask */
/* SUSPend Mask */
/* RESET Mask   */
/* Start Of Frame Mask */
/* Expected Start Of Frame Mask */
/* RESUME request */
/* Force SUSPend */
/* Low-power MODE */
/* Power DoWN */
/* Force USB RESet */
/* *****************************************************************************/
/*                FNR Frame Number Register bit definitions                   */
/* *****************************************************************************/
/* status of D+ data line */
/* status of D- data line */
/* LoCKed */
/* Lost SOF */
/* Frame Number */
/* *****************************************************************************/
/*               DADDR Device ADDRess bit definitions                         */
/* *****************************************************************************/
/* *****************************************************************************/
/*                            Endpoint register                               */
/* *****************************************************************************/
/* bit positions */
/* EndPoint Correct TRansfer RX */
/* EndPoint Data TOGGLE RX */
/* EndPoint RX STATus bit field */
/* EndPoint SETUP */
/* EndPoint TYPE */
/* EndPoint KIND */
/* EndPoint Correct TRansfer TX */
/* EndPoint Data TOGGLE TX */
/* EndPoint TX STATus bit field */
/* EndPoint ADDRess FIELD */
/* EndPoint REGister MASK (no toggle fields) */
/* EP_TYPE[1:0] EndPoint TYPE */
/* EndPoint TYPE Mask */
/* EndPoint BULK */
/* EndPoint CONTROL */
/* EndPoint ISOCHRONOUS */
/* EndPoint INTERRUPT */
/* EP_KIND EndPoint KIND */
/* STAT_TX[1:0] STATus for TX transfer */
/* EndPoint TX DISabled */
/* EndPoint TX STALLed */
/* EndPoint TX NAKed */
/* EndPoint TX VALID */
/* EndPoint TX Data TOGgle bit1 */
/* EndPoint TX Data TOGgle bit2 */
/* STAT_RX[1:0] STATus for RX transfer */
/* EndPoint RX DISabled */
/* EndPoint RX STALLed */
/* EndPoint RX NAKed */
/* EndPoint RX VALID */
/* EndPoint RX Data TOGgle bit1 */
/* EndPoint RX Data TOGgle bit1 */
/* Exported macro ------------------------------------------------------------*/
/* SetCNTR */
/* SetISTR */
/* SetDADDR */
/* SetBTABLE */
/* GetCNTR */
/* GetISTR */
/* GetFNR */
/* GetDADDR */
/* GetBTABLE ; clear low-order bits explicitly to avoid problems in gcc 5.x */
/* SetENDPOINT */
/* GetENDPOINT */
/* ******************************************************************************
* Macro Name     : SetEPType
* Description    : sets the type in the endpoint register(bits EP_TYPE[1:0])
* Input          : bEpNum: Endpoint Number. 
*                  wType											 
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : GetEPType
* Description    : gets the type in the endpoint register(bits EP_TYPE[1:0]) 
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint Type
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : SetEPTxStatus
* Description    : sets the status for tx transfer (bits STAT_TX[1:0]).
* Input          : bEpNum: Endpoint Number. 
*                  wState: new state
* Output         : None.
* Return         : None.
*******************************************************************************/
/* toggle first bit ? */
/* toggle second bit ?  */
/* _SetEPTxStatus */
/* ******************************************************************************
* Macro Name     : SetEPRxStatus
* Description    : sets the status for rx transfer (bits STAT_TX[1:0])
* Input          : bEpNum: Endpoint Number. 
*                  wState: new state.
* Output         : None.
* Return         : None.
*******************************************************************************/
/* toggle first bit ? */
/* toggle second bit ? */
/* _SetEPRxStatus */
/* ******************************************************************************
* Macro Name     : SetEPRxTxStatus
* Description    : sets the status for rx & tx (bits STAT_TX[1:0] & STAT_RX[1:0])
* Input          : bEpNum: Endpoint Number. 
*                  wStaterx: new state.
*                  wStatetx: new state.
* Output         : None.
* Return         : None.
*******************************************************************************/
/* toggle first bit ? */
/* toggle second bit ? */
/* toggle first bit ? */
/* toggle second bit ?  */
/* _SetEPRxTxStatus */
/* ******************************************************************************
* Macro Name     : GetEPTxStatus / GetEPRxStatus 
* Description    : gets the status for tx/rx transfer (bits STAT_TX[1:0]
*                  /STAT_RX[1:0])
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : status .
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : SetEPTxValid / SetEPRxValid 
* Description    : sets directly the VALID tx/rx-status into the enpoint register
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : GetTxStallStatus / GetRxStallStatus.
* Description    : checks stall condition in an endpoint.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : TRUE = endpoint in stall condition.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : SetEP_KIND / ClearEP_KIND.
* Description    : set & clear EP_KIND bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : Set_Status_Out / Clear_Status_Out.
* Description    : Sets/clears directly STATUS_OUT bit in the endpoint register.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : SetEPDoubleBuff / ClearEPDoubleBuff.
* Description    : Sets/clears directly EP_KIND bit in the endpoint register.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : ClearEP_CTR_RX / ClearEP_CTR_TX.
* Description    : Clears bit CTR_RX / CTR_TX in the endpoint register.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : ToggleDTOG_RX / ToggleDTOG_TX .
* Description    : Toggles DTOG_RX / DTOG_TX bit in the endpoint register.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : ClearDTOG_RX / ClearDTOG_TX.
* Description    : Clears DTOG_RX / DTOG_TX bit in the endpoint register.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : SetEPAddress.
* Description    : Sets address in an endpoint register.
* Input          : bEpNum: Endpoint Number.
*                  bAddr: Address. 
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : GetEPAddress.
* Description    : Gets address in an endpoint register.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : SetEPTxAddr / SetEPRxAddr.
* Description    : sets address of the tx/rx buffer.
* Input          : bEpNum: Endpoint Number.
*                  wAddr: address to be set (must be word aligned).
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : GetEPTxAddr / GetEPRxAddr.
* Description    : Gets address of the tx/rx buffer.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : address of the buffer.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : SetEPCountRxReg.
* Description    : Sets counter of rx buffer with no. of blocks.
* Input          : pdwReg: pointer to counter.
*                  wCount: Counter.
* Output         : None.
* Return         : None.
*******************************************************************************/
/* _BlocksOf32 */
/* _BlocksOf2 */
/* _SetEPCountRxReg */
/* ******************************************************************************
* Macro Name     : SetEPTxCount / SetEPRxCount.
* Description    : sets counter for the tx/rx buffer.
* Input          : bEpNum: endpoint number.
*                  wCount: Counter value.
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : GetEPTxCount / GetEPRxCount.
* Description    : gets counter of the tx buffer.
* Input          : bEpNum: endpoint number.
* Output         : None.
* Return         : Counter value.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : SetEPDblBuf0Addr / SetEPDblBuf1Addr.
* Description    : Sets buffer 0/1 address in a double buffer endpoint.
* Input          : bEpNum: endpoint number.
*                : wBuf0Addr: buffer 0 address.
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : SetEPDblBuffAddr.
* Description    : Sets addresses in a double buffer endpoint.
* Input          : bEpNum: endpoint number.
*                : wBuf0Addr: buffer 0 address.
*                : wBuf1Addr = buffer 1 address.
* Output         : None.
* Return         : None.
*******************************************************************************/
/* _SetEPDblBuffAddr */
/* ******************************************************************************
* Macro Name     : GetEPDblBuf0Addr / GetEPDblBuf1Addr.
* Description    : Gets buffer 0/1 address of a double buffer endpoint.
* Input          : bEpNum: endpoint number.
* Output         : None.
* Return         : None.
*******************************************************************************/
/* ******************************************************************************
* Macro Name     : SetEPDblBuffCount / SetEPDblBuf0Count / SetEPDblBuf1Count.
* Description    : Gets buffer 0/1 address of a double buffer endpoint.
* Input          : bEpNum: endpoint number.
*                : bDir: endpoint dir  EP_DBUF_OUT = OUT 
*                                      EP_DBUF_IN  = IN 
*                : wCount: Counter value    
* Output         : None.
* Return         : None.
*******************************************************************************/
/* OUT endpoint */
/* IN endpoint */
/* SetEPDblBuf0Count*/
/* OUT endpoint */
/* IN endpoint */
/* SetEPDblBuf1Count */
/* _SetEPDblBuffCount  */
/* ******************************************************************************
* Macro Name     : GetEPDblBuf0Count / GetEPDblBuf1Count.
* Description    : Gets buffer 0/1 rx/tx counter for double buffering.
* Input          : bEpNum: endpoint number.
* Output         : None.
* Return         : None.
*******************************************************************************/
/* External variables --------------------------------------------------------*/
/* ISTR register last read value */
/* Exported functions ------------------------------------------------------- */
/*wRegValue*/
/*wRegValue*/
/*wRegValue*/
/*wRegValue*/
/*wRegValue*/
/*bEpNum*/
/*wRegValue*/
/*bEpNum*/
/*bEpNum*/
/*wType*/
/*bEpNum*/
/*bEpNum*/
/*wState*/
/*bEpNum*/
/*wState*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bAddr*/
/*bEpNum*/
/*bEpNum*/
/*wAddr*/
/*bEpNum*/
/*wAddr*/
/*bEpNum*/
/*bEpNum*/
/*pdwReg*/
/*wCount*/
/*bEpNum*/
/*wCount*/
/*bEpNum*/
/*wCount*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*wBuf0Addr*/
/*bEpNum*/
/*wBuf1Addr*/
/*bEpNum*/
/*wBuf0Addr*/
/*wBuf1Addr*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bDir*/
/*wCount*/
/*bEpNum*/
/*bDir*/
/*wCount*/
/*bEpNum*/
/*bDir*/
/*wCount*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/*bEpNum*/
/* ******************************************************************************
* Function Name  : ByteSwap
* Description    : Swap two byte in a word.
* Input          : wSwW: word to Swap.
* Output         : None.
* Return         : resulted word.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn ByteSwap(mut wSwW: uint16_t) -> uint16_t {
    let mut bTemp: uint8_t = 0;
    let mut wRet: uint16_t = 0;
    bTemp = (wSwW as libc::c_int & 0xff as libc::c_int) as uint8_t;
    wRet =
        (wSwW as libc::c_int >> 8 as libc::c_int |
             (bTemp as uint16_t as libc::c_int) << 8 as libc::c_int) as
            uint16_t;
    return wRet;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
