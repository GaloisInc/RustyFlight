use ::libc;
extern "C" {
    /* *
  ******************************************************************************
  * @file    usb_core.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Standard protocol processing functions prototypes
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
    /* 0 */
    /* 1 */
    /* 2 */
    /* 3 */
    /* 4 */
    /* 5 */
    /* 7 */
    /* 8 */
    /* 9 */
    /* 10 */
    /* The state machine states of a control pipe */
    /* All the request process routines return a value of this type
   If the return value is not SUCCESS or NOT_READY,
   the software will STALL the correspond endpoint */
    /* Process successfully */
    /* The process has not been finished, endpoint will be
                         NAK to further request */
    /*-*-*-*-*-*-*-*-*-*-* Definitions for endpoint level -*-*-*-*-*-*-*-*-*-*-*-*/
    /* When send data out of the device,
   CopyData() is used to get data buffer 'Length' bytes data
   if Length is 0,
    CopyData() returns the total length of the data
    if the request is not supported, returns 0
    (NEW Feature )
     if CopyData() returns -1, the calling routine should not proceed
     further and will resume the SETUP process by the class device
   if Length is not 0,
    CopyData() returns a pointer to indicate the data location
   Usb_wLength is the data remain to be sent,
   Usb_wOffset is the Offset of original data
  When receive data from the host,
   CopyData() is used to get user data buffer which is capable
   of Length bytes data to copy data from the endpoint buffer.
   if Length is 0,
    CopyData() returns the available data length,
   if Length is not 0,
    CopyData() returns user buffer address
   Usb_rLength is the data remain to be received,
   Usb_rPointer is the Offset of data buffer
  */
    /*-*-*-*-*-*-*-*-*-*-*-* Definitions for device level -*-*-*-*-*-*-*-*-*-*-*-*/
    /* Number of endpoints that are used */
    /* Number of configuration available */
    /* bmRequestType */
    /* bRequest */
    /* wValue */
    /* wIndex */
    /* wLength */
    /* of type CONTROL_STATE */
    /* Selected configuration */
    /* Selected interface of current configuration */
    /* Selected Alternate Setting of current
                                     interface*/
    /* Initialize the device */
    /* Reset routine of this device */
    /* Device dependent process after the status stage */
    /* Procedure of process on setup stage of a class specified request with data stage */
  /* All class specified requests with data stage are processed in Class_Data_Setup
   Class_Data_Setup()
    responses to check all special requests and fills ENDPOINT_INFO
    according to the request
    If IN tokens are expected, then wLength & wOffset will be filled
    with the total transferring bytes and the starting position
    If OUT tokens are expected, then rLength & rOffset will be filled
    with the total expected bytes and the starting position in the buffer

    If the request is valid, Class_Data_Setup returns SUCCESS, else UNSUPPORT

   CAUTION:
    Since GET_CONFIGURATION & GET_INTERFACE are highly related to
    the individual classes, they will be checked and processed here.
  */
    /* Procedure of process on setup stage of a class specified request without data stage */
  /* All class specified requests without data stage are processed in Class_NoData_Setup
   Class_NoData_Setup
    responses to check all special requests and perform the request

   CAUTION:
    Since SET_CONFIGURATION & SET_INTERFACE are highly related to
    the individual classes, they will be checked and processed here.
  */
    /*Class_Get_Interface_Setting
   This function is used by the file usb_core.c to test if the selected Interface
   and Alternate Setting (uint8_t Interface, uint8_t AlternateSetting) are supported by
   the application.
   This function is writing by user. It should return "SUCCESS" if the Interface
   and Alternate Setting are supported by the application or "UNSUPPORT" if they
   are not supported. */
    /* This field is not used in current library version. It is kept only for 
   compatibility with previous versions */
    /* Get Configuration */
    /* Set Configuration */
    /* Get Interface */
    /* Set Interface */
    /* Get Status */
    /* Clear Feature */
    /* Set Endpoint Feature */
    /* Set Device Feature */
    /* Set Device Address */
    /* Exported constants --------------------------------------------------------*/
    /* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
    #[no_mangle]
    fn Out0_Process() -> uint8_t;
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
    #[no_mangle]
    static mut wIstr: uint16_t;
    #[no_mangle]
    fn Setup0_Process() -> uint8_t;
    #[no_mangle]
    fn In0_Process() -> uint8_t;
    #[no_mangle]
    static mut EPindex: uint8_t;
    /* Extern variables ----------------------------------------------------------*/
    #[no_mangle]
    static mut pEpInt_IN: [Option<unsafe extern "C" fn() -> ()>; 7];
    /*  Handles IN  interrupts   */
    #[no_mangle]
    static mut pEpInt_OUT: [Option<unsafe extern "C" fn() -> ()>; 7];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  ******************************************************************************
  * @file    usb_int.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Endpoint CTR (Low and High) interrupt's service routines
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
#[no_mangle]
pub static mut SaveRState: uint16_t = 0;
#[no_mangle]
pub static mut SaveTState: uint16_t = 0;
/*  Handles OUT interrupts   */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ******************************************************************************
* Function Name  : CTR_LP.
* Description    : Low priority Endpoint Correct Transfer interrupt's service
*                  routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CTR_LP() {
    let mut wEPVal: uint16_t = 0 as libc::c_int as uint16_t;
    loop 
         /* stay in loop while pending interrupts */
         {
        ::core::ptr::write_volatile(&mut wIstr as *mut uint16_t,
                                    *((0x40005c00 as libc::c_long +
                                           0x44 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t);
        if !(::core::ptr::read_volatile::<uint16_t>(&wIstr as *const uint16_t)
                 as libc::c_int & 0x8000 as libc::c_int != 0 as libc::c_int) {
            break ;
        }
        /* extract highest priority endpoint number */
        EPindex = (wIstr as libc::c_int & 0xf as libc::c_int) as uint8_t;
        if EPindex as libc::c_int == 0 as libc::c_int {
            /* if(EPindex == 0) else */
            ::core::ptr::write_volatile(&mut SaveRState as *mut uint16_t,
                                        *(0x40005c00 as libc::c_long as
                                              *mut libc::c_uint).offset(0 as
                                                                            libc::c_int
                                                                            as
                                                                            uint8_t
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            isize)
                                            as
                                            uint16_t); /* if(EPindex == 0) */
            ::core::ptr::write_volatile(&mut SaveTState as *mut uint16_t,
                                        (SaveRState as libc::c_int &
                                             0x30 as libc::c_int) as
                                            uint16_t);
            ::core::ptr::write_volatile(&mut SaveRState as *mut uint16_t,
                                        (::core::ptr::read_volatile::<uint16_t>(&SaveRState
                                                                                    as
                                                                                    *const uint16_t)
                                             as libc::c_int &
                                             0x3000 as libc::c_int) as
                                            uint16_t as uint16_t);
            let mut _wRegVal: uint32_t = 0;
            _wRegVal =
                (*(0x40005c00 as libc::c_long as
                       *mut libc::c_uint).offset(0 as libc::c_int as uint8_t
                                                     as libc::c_int as isize)
                     as uint16_t as libc::c_int &
                     (0x3000 as libc::c_int |
                          (0x8000 as libc::c_int | 0x800 as libc::c_int |
                               0x600 as libc::c_int | 0x100 as libc::c_int |
                               0x80 as libc::c_int | 0xf as libc::c_int) |
                          0x30 as libc::c_int)) as uint32_t;
            if 0x1000 as libc::c_int & 0x2000 as libc::c_int !=
                   0 as libc::c_int {
                _wRegVal ^= 0x1000 as libc::c_int as libc::c_uint
            }
            if 0x2000 as libc::c_int & 0x2000 as libc::c_int !=
                   0 as libc::c_int {
                _wRegVal ^= 0x2000 as libc::c_int as libc::c_uint
            }
            if 0x10 as libc::c_int & 0x20 as libc::c_int != 0 as libc::c_int {
                _wRegVal ^= 0x10 as libc::c_int as libc::c_uint
            }
            if 0x20 as libc::c_int & 0x20 as libc::c_int != 0 as libc::c_int {
                _wRegVal ^= 0x20 as libc::c_int as libc::c_uint
            }
            ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                             *mut libc::c_uint).offset(0 as
                                                                           libc::c_int
                                                                           as
                                                                           uint8_t
                                                                           as
                                                                           libc::c_int
                                                                           as
                                                                           isize),
                                        (_wRegVal as uint16_t as libc::c_int |
                                             0x8000 as libc::c_int |
                                             0x80 as libc::c_int) as
                                            libc::c_uint);
            /* Decode and service control endpoint interrupt */
      /* calling related service routine */
      /* (Setup0_Process, In0_Process, Out0_Process) */
            /* save RX & TX status */
      /* and set both to NAK */
            /* DIR bit = origin of the interrupt */
            if wIstr as libc::c_int & 0x10 as libc::c_int == 0 as libc::c_int
               {
                /* DIR = 0 */
                /* DIR = 0      => IN  int */
        /* DIR = 0 implies that (EP_CTR_TX = 1) always  */
                ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                 *mut libc::c_uint).offset(0
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint8_t
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               isize),
                                            (*(0x40005c00 as libc::c_long as
                                                   *mut libc::c_uint).offset(0
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 uint8_t
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize)
                                                 as uint16_t as libc::c_int &
                                                 0xff7f as libc::c_int &
                                                 (0x8000 as libc::c_int |
                                                      0x800 as libc::c_int |
                                                      0x600 as libc::c_int |
                                                      0x100 as libc::c_int |
                                                      0x80 as libc::c_int |
                                                      0xf as libc::c_int)) as
                                                libc::c_uint);
                In0_Process();
                /* before terminate set Tx & Rx status */
                let mut _wRegVal_0: uint32_t = 0;
                _wRegVal_0 =
                    (*(0x40005c00 as libc::c_long as
                           *mut libc::c_uint).offset(0 as libc::c_int as
                                                         uint8_t as
                                                         libc::c_int as isize)
                         as uint16_t as libc::c_int &
                         (0x3000 as libc::c_int |
                              (0x8000 as libc::c_int | 0x800 as libc::c_int |
                                   0x600 as libc::c_int | 0x100 as libc::c_int
                                   | 0x80 as libc::c_int | 0xf as libc::c_int)
                              | 0x30 as libc::c_int)) as uint32_t;
                if 0x1000 as libc::c_int & SaveRState as libc::c_int !=
                       0 as libc::c_int {
                    _wRegVal_0 ^= 0x1000 as libc::c_int as libc::c_uint
                }
                if 0x2000 as libc::c_int & SaveRState as libc::c_int !=
                       0 as libc::c_int {
                    _wRegVal_0 ^= 0x2000 as libc::c_int as libc::c_uint
                }
                if 0x10 as libc::c_int & SaveTState as libc::c_int !=
                       0 as libc::c_int {
                    _wRegVal_0 ^= 0x10 as libc::c_int as libc::c_uint
                }
                if 0x20 as libc::c_int & SaveTState as libc::c_int !=
                       0 as libc::c_int {
                    _wRegVal_0 ^= 0x20 as libc::c_int as libc::c_uint
                }
                ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                 *mut libc::c_uint).offset(0
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint8_t
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               isize),
                                            (_wRegVal_0 as uint16_t as
                                                 libc::c_int |
                                                 0x8000 as libc::c_int |
                                                 0x80 as libc::c_int) as
                                                libc::c_uint);
                return
            } else {
                /* DIR = 1 */
                /* DIR = 1 & CTR_RX       => SETUP or OUT int */
        /* DIR = 1 & (CTR_TX | CTR_RX) => 2 int pending */
                ::core::ptr::write_volatile(&mut wEPVal as *mut uint16_t,
                                            *(0x40005c00 as libc::c_long as
                                                  *mut libc::c_uint).offset(0
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                uint8_t
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                                as
                                                uint16_t); /* SETUP bit kept frozen while CTR_RX = 1 */
                if wEPVal as libc::c_int & 0x800 as libc::c_int !=
                       0 as libc::c_int {
                    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                     *mut libc::c_uint).offset(0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint8_t
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   isize),
                                                (*(0x40005c00 as libc::c_long
                                                       as
                                                       *mut libc::c_uint).offset(0
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     uint8_t
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     isize)
                                                     as uint16_t as
                                                     libc::c_int &
                                                     0x7fff as libc::c_int &
                                                     (0x8000 as libc::c_int |
                                                          0x800 as libc::c_int
                                                          |
                                                          0x600 as libc::c_int
                                                          |
                                                          0x100 as libc::c_int
                                                          |
                                                          0x80 as libc::c_int
                                                          |
                                                          0xf as libc::c_int))
                                                    as libc::c_uint);
                    Setup0_Process();
                    /* before terminate set Tx & Rx status */
                    let mut _wRegVal_1: uint32_t = 0;
                    _wRegVal_1 =
                        (*(0x40005c00 as libc::c_long as
                               *mut libc::c_uint).offset(0 as libc::c_int as
                                                             uint8_t as
                                                             libc::c_int as
                                                             isize) as
                             uint16_t as libc::c_int &
                             (0x3000 as libc::c_int |
                                  (0x8000 as libc::c_int |
                                       0x800 as libc::c_int |
                                       0x600 as libc::c_int |
                                       0x100 as libc::c_int |
                                       0x80 as libc::c_int |
                                       0xf as libc::c_int) |
                                  0x30 as libc::c_int)) as uint32_t;
                    if 0x1000 as libc::c_int & SaveRState as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal_1 ^= 0x1000 as libc::c_int as libc::c_uint
                    }
                    if 0x2000 as libc::c_int & SaveRState as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal_1 ^= 0x2000 as libc::c_int as libc::c_uint
                    }
                    if 0x10 as libc::c_int & SaveTState as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal_1 ^= 0x10 as libc::c_int as libc::c_uint
                    }
                    if 0x20 as libc::c_int & SaveTState as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal_1 ^= 0x20 as libc::c_int as libc::c_uint
                    }
                    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                     *mut libc::c_uint).offset(0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint8_t
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   isize),
                                                (_wRegVal_1 as uint16_t as
                                                     libc::c_int |
                                                     0x8000 as libc::c_int |
                                                     0x80 as libc::c_int) as
                                                    libc::c_uint);
                    return
                } else {
                    if wEPVal as libc::c_int & 0x8000 as libc::c_int !=
                           0 as libc::c_int {
                        ::core::ptr::write_volatile((0x40005c00 as
                                                         libc::c_long as
                                                         *mut libc::c_uint).offset(0
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       uint8_t
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       isize),
                                                    (*(0x40005c00 as
                                                           libc::c_long as
                                                           *mut libc::c_uint).offset(0
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         uint8_t
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         isize)
                                                         as uint16_t as
                                                         libc::c_int &
                                                         0x7fff as libc::c_int
                                                         &
                                                         (0x8000 as
                                                              libc::c_int |
                                                              0x800 as
                                                                  libc::c_int
                                                              |
                                                              0x600 as
                                                                  libc::c_int
                                                              |
                                                              0x100 as
                                                                  libc::c_int
                                                              |
                                                              0x80 as
                                                                  libc::c_int
                                                              |
                                                              0xf as
                                                                  libc::c_int))
                                                        as libc::c_uint);
                        Out0_Process();
                        /* before terminate set Tx & Rx status */
                        let mut _wRegVal_2: uint32_t = 0;
                        _wRegVal_2 =
                            (*(0x40005c00 as libc::c_long as
                                   *mut libc::c_uint).offset(0 as libc::c_int
                                                                 as uint8_t as
                                                                 libc::c_int
                                                                 as isize) as
                                 uint16_t as libc::c_int &
                                 (0x3000 as libc::c_int |
                                      (0x8000 as libc::c_int |
                                           0x800 as libc::c_int |
                                           0x600 as libc::c_int |
                                           0x100 as libc::c_int |
                                           0x80 as libc::c_int |
                                           0xf as libc::c_int) |
                                      0x30 as libc::c_int)) as uint32_t;
                        if 0x1000 as libc::c_int & SaveRState as libc::c_int
                               != 0 as libc::c_int {
                            _wRegVal_2 ^=
                                0x1000 as libc::c_int as libc::c_uint
                        }
                        if 0x2000 as libc::c_int & SaveRState as libc::c_int
                               != 0 as libc::c_int {
                            _wRegVal_2 ^=
                                0x2000 as libc::c_int as libc::c_uint
                        }
                        if 0x10 as libc::c_int & SaveTState as libc::c_int !=
                               0 as libc::c_int {
                            _wRegVal_2 ^= 0x10 as libc::c_int as libc::c_uint
                        }
                        if 0x20 as libc::c_int & SaveTState as libc::c_int !=
                               0 as libc::c_int {
                            _wRegVal_2 ^= 0x20 as libc::c_int as libc::c_uint
                        }
                        ::core::ptr::write_volatile((0x40005c00 as
                                                         libc::c_long as
                                                         *mut libc::c_uint).offset(0
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       uint8_t
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       isize),
                                                    (_wRegVal_2 as uint16_t as
                                                         libc::c_int |
                                                         0x8000 as libc::c_int
                                                         |
                                                         0x80 as libc::c_int)
                                                        as libc::c_uint);
                        return
                    }
                }
            }
        } else {
            /* Decode and service non control endpoints interrupt  */
            /* process related endpoint register */
            ::core::ptr::write_volatile(&mut wEPVal as *mut uint16_t,
                                        *(0x40005c00 as libc::c_long as
                                              *mut libc::c_uint).offset(EPindex
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            isize)
                                            as uint16_t);
            /* if((wEPVal & EP_CTR_TX) != 0) */
            if wEPVal as libc::c_int & 0x8000 as libc::c_int !=
                   0 as libc::c_int { /* if((wEPVal & EP_CTR_RX) */
                /* clear int flag */
                ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                 *mut libc::c_uint).offset(EPindex
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               isize),
                                            (*(0x40005c00 as libc::c_long as
                                                   *mut libc::c_uint).offset(EPindex
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize)
                                                 as uint16_t as libc::c_int &
                                                 0x7fff as libc::c_int &
                                                 (0x8000 as libc::c_int |
                                                      0x800 as libc::c_int |
                                                      0x600 as libc::c_int |
                                                      0x100 as libc::c_int |
                                                      0x80 as libc::c_int |
                                                      0xf as libc::c_int)) as
                                                libc::c_uint);
                /* call OUT service function */
                Some((*pEpInt_OUT.as_mut_ptr().offset((EPindex as libc::c_int
                                                           - 1 as libc::c_int)
                                                          as
                                                          isize)).expect("non-null function pointer")).expect("non-null function pointer")();
            }
            if wEPVal as libc::c_int & 0x80 as libc::c_int != 0 as libc::c_int
               {
                /* clear int flag */
                ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                 *mut libc::c_uint).offset(EPindex
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               isize),
                                            (*(0x40005c00 as libc::c_long as
                                                   *mut libc::c_uint).offset(EPindex
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize)
                                                 as uint16_t as libc::c_int &
                                                 0xff7f as libc::c_int &
                                                 (0x8000 as libc::c_int |
                                                      0x800 as libc::c_int |
                                                      0x600 as libc::c_int |
                                                      0x100 as libc::c_int |
                                                      0x80 as libc::c_int |
                                                      0xf as libc::c_int)) as
                                                libc::c_uint);
                /* call IN service function */
                Some((*pEpInt_IN.as_mut_ptr().offset((EPindex as libc::c_int -
                                                          1 as libc::c_int) as
                                                         isize)).expect("non-null function pointer")).expect("non-null function pointer")();
            }
        }
    };
    /* while(...) */
}
/* *
  ******************************************************************************
  * @file    usb_int.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Endpoint CTR (Low and High) interrupt's service routines prototypes
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
* Function Name  : CTR_HP.
* Description    : High Priority Endpoint Correct Transfer interrupt's service 
*                  routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CTR_HP() {
    let mut wEPVal: uint32_t =
        0 as libc::c_int as uint32_t; /* clear CTR flag */
    loop  {
        ::core::ptr::write_volatile(&mut wIstr as *mut uint16_t,
                                    *((0x40005c00 as libc::c_long +
                                           0x44 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t);
        if !(::core::ptr::read_volatile::<uint16_t>(&wIstr as *const uint16_t)
                 as libc::c_int & 0x8000 as libc::c_int != 0 as libc::c_int) {
            break ;
        }
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                         0x44 as libc::c_int as libc::c_long)
                                        as *mut libc::c_uint,
                                    !(0x8000 as libc::c_int) as uint16_t as
                                        libc::c_uint);
        /* if((wEPVal & EP_CTR_TX) != 0) */
        EPindex = (wIstr as libc::c_int & 0xf as libc::c_int) as uint8_t;
        wEPVal =
            *(0x40005c00 as libc::c_long as
                  *mut libc::c_uint).offset(EPindex as libc::c_int as isize)
                as uint16_t as uint32_t;
        if wEPVal & 0x8000 as libc::c_int as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            /* extract highest priority endpoint number */
            /* process related endpoint register */
            ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                             *mut libc::c_uint).offset(EPindex
                                                                           as
                                                                           libc::c_int
                                                                           as
                                                                           isize),
                                        (*(0x40005c00 as libc::c_long as
                                               *mut libc::c_uint).offset(EPindex
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             isize)
                                             as uint16_t as libc::c_int &
                                             0x7fff as libc::c_int &
                                             (0x8000 as libc::c_int |
                                                  0x800 as libc::c_int |
                                                  0x600 as libc::c_int |
                                                  0x100 as libc::c_int |
                                                  0x80 as libc::c_int |
                                                  0xf as libc::c_int)) as
                                            libc::c_uint); /* if((wEPVal & EP_CTR_RX) */
            /* clear int flag */
            /* call OUT service function */
            Some((*pEpInt_OUT.as_mut_ptr().offset((EPindex as libc::c_int -
                                                       1 as libc::c_int) as
                                                      isize)).expect("non-null function pointer")).expect("non-null function pointer")();
        } else if wEPVal & 0x80 as libc::c_int as libc::c_uint !=
                      0 as libc::c_int as libc::c_uint {
            /* clear int flag */
            ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                             *mut libc::c_uint).offset(EPindex
                                                                           as
                                                                           libc::c_int
                                                                           as
                                                                           isize),
                                        (*(0x40005c00 as libc::c_long as
                                               *mut libc::c_uint).offset(EPindex
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             isize)
                                             as uint16_t as libc::c_int &
                                             0xff7f as libc::c_int &
                                             (0x8000 as libc::c_int |
                                                  0x800 as libc::c_int |
                                                  0x600 as libc::c_int |
                                                  0x100 as libc::c_int |
                                                  0x80 as libc::c_int |
                                                  0xf as libc::c_int)) as
                                            libc::c_uint);
            /* call IN service function */
            Some((*pEpInt_IN.as_mut_ptr().offset((EPindex as libc::c_int -
                                                      1 as libc::c_int) as
                                                     isize)).expect("non-null function pointer")).expect("non-null function pointer")();
        }
    };
    /* while(...) */
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
