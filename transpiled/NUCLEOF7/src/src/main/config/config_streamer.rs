use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    /* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
    /* * @defgroup FLASHEx_Exported_Constants FLASH Exported Constants
  * @{
  */
    /* * @defgroup FLASHEx_Type_Erase FLASH Type Erase
  * @{
  */
    /* !< Sectors erase only          */
    /* !< Flash Mass erase activation */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_Voltage_Range FLASH Voltage Range
  * @{
  */
    /* !< Device operating range: 1.8V to 2.1V                */
    /* !< Device operating range: 2.1V to 2.7V                */
    /* !< Device operating range: 2.7V to 3.6V                */
    /* !< Device operating range: 2.7V to 3.6V + External Vpp */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_WRP_State FLASH WRP State
  * @{
  */
    /* !< Disable the write protection of the desired bank 1 sectors */
    /* !< Enable the write protection of the desired bank 1 sectors  */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_Option_Type FLASH Option Type
  * @{
  */
    /* !< WRP option byte configuration  */
    /* !< RDP option byte configuration  */
    /* !< USER option byte configuration */
    /* !< BOR option byte configuration  */
    /* !< Boot 0 Address configuration   */
    /* !< Boot 1 Address configuration   */
    /* FLASH_OPTCR2_PCROP */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_Option_Bytes_Read_Protection FLASH Option Bytes Read Protection
  * @{
  */
    /* !< Warning: When enabling read protection level 2 
                                                  it s no more possible to go back to level 1 or 0 */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_Option_Bytes_WWatchdog FLASH Option Bytes WWatchdog
  * @{
  */
    /* !< Software WWDG selected */
    /* !< Hardware WWDG selected */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_Option_Bytes_IWatchdog FLASH Option Bytes IWatchdog
  * @{
  */
    /* !< Software IWDG selected */
    /* !< Hardware IWDG selected */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_Option_Bytes_nRST_STOP FLASH Option Bytes nRST_STOP
  * @{
  */
    /* !< No reset generated when entering in STOP */
    /* !< Reset generated when entering in STOP    */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_Option_Bytes_nRST_STDBY FLASH Option Bytes nRST_STDBY
  * @{
  */
    /* !< No reset generated when entering in STANDBY */
    /* !< Reset generated when entering in STANDBY    */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_Option_Bytes_IWDG_FREEZE_STOP FLASH IWDG Counter Freeze in STOP
  * @{
  */
    /* !< Freeze IWDG counter in STOP mode */
    /* !< IWDG counter active in STOP mode */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_Option_Bytes_IWDG_FREEZE_SANDBY FLASH IWDG Counter Freeze in STANDBY
  * @{
  */
    /* !< Freeze IWDG counter in STANDBY mode */
    /* !< IWDG counter active in STANDBY mode */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_BOR_Reset_Level FLASH BOR Reset Level
  * @{
  */
    /* !< Supply voltage ranges from 2.70 to 3.60 V */
    /* !< Supply voltage ranges from 2.40 to 2.70 V */
    /* !< Supply voltage ranges from 2.10 to 2.40 V */
    /* !< Supply voltage ranges from 1.62 to 2.10 V */
    /* *
  * @}
  */
    /* FLASH_OPTCR_nDBOOT */
    /* FLASH_OPTCR_nDBANK */
    /* * @defgroup FLASHEx_Boot_Address FLASH Boot Address
  * @{
  */
    /* !< Boot from ITCM RAM (0x00000000)                 */
    /* !< Boot from System memory bootloader (0x00100000) */
    /* !< Boot from Flash on ITCM interface (0x00200000)  */
    /* !< Boot from Flash on AXIM interface (0x08000000)  */
    /* !< Boot from DTCM RAM (0x20000000)                 */
    /* !< Boot from SRAM1 (0x20010000)                    */
    /* !< Boot from SRAM2 (0x2004C000)                    */
    /* SRAM2_BASE == 0x2003C000U */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Latency FLASH Latency
  * @{
  */
    /* !< FLASH Zero Latency cycle      */
    /* !< FLASH One Latency cycle       */
    /* !< FLASH Two Latency cycles      */
    /* !< FLASH Three Latency cycles    */
    /* !< FLASH Four Latency cycles     */
    /* !< FLASH Five Latency cycles     */
    /* !< FLASH Six Latency cycles      */
    /* !< FLASH Seven Latency cycles    */
    /* !< FLASH Eight Latency cycles    */
    /* !< FLASH Nine Latency cycles     */
    /* !< FLASH Ten Latency cycles      */
    /* !< FLASH Eleven Latency cycles   */
    /* !< FLASH Twelve Latency cycles   */
    /* !< FLASH Thirteen Latency cycles */
    /* !< FLASH Fourteen Latency cycles */
    /* !< FLASH Fifteen Latency cycles  */
    /* *
  * @}
  */
    /* FLASH_OPTCR_nDBANK */
    /* * @defgroup FLASHEx_MassErase_bit FLASH Mass Erase bit
  * @{
  */
    /* !< only 1 MER bit */
    /* FLASH_OPTCR_nDBANK */
    /* *
  * @}
  */
    /* * @defgroup FLASHEx_Sectors FLASH Sectors
  * @{
  */
    /* FLASH_SECTOR_TOTAL == 24 */
    /* *
  * @}
  */
    /* FLASH_SECTOR_TOTAL == 24 */
    /* * @defgroup FLASHEx_Option_Bytes_Write_Protection FLASH Option Bytes Write Protection
  * @{
  */
    /* !< Write protection of Sector0     */
    /* !< Write protection of Sector1     */
    /* !< Write protection of Sector2     */
    /* !< Write protection of Sector3     */
    /* !< Write protection of Sector4     */
    /* !< Write protection of Sector5     */
    /* !< Write protection of Sector6     */
    /* !< Write protection of Sector7     */
    /* !< Write protection of all Sectors */
    /* *
  * @}
  */
    /* FLASH_SECTOR_TOTAL == 8 */
    /* FLASH_OPTCR2_PCROP */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup FLASH_Exported_Macros FLASH Exported Macros
  * @{
  */
/* *
  * @brief  Calculate the FLASH Boot Base Adress (BOOT_ADD0 or BOOT_ADD1)
  * @note   Returned value BOOT_ADDx[15:0] corresponds to boot address [29:14].
  * @param  __ADDRESS__: FLASH Boot Address (in the range 0x0000 0000 to 0x2004 FFFF with a granularity of 16KB)
  * @retval The FLASH Boot Base Adress
  */
    /* *
  * @}
  */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup FLASHEx_Exported_Functions
  * @{
  */
    /* * @addtogroup FLASHEx_Exported_Functions_Group1
  * @{
  */
/* Extension Program operation functions  *************************************/
    #[no_mangle]
    fn HAL_FLASHEx_Erase(pEraseInit: *mut FLASH_EraseInitTypeDef,
                         SectorError: *mut uint32_t) -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* Exported constants --------------------------------------------------------*/
/* * @defgroup FLASH_Exported_Constants FLASH Exported Constants
  * @{
  */
    /* * @defgroup FLASH_Error_Code FLASH Error Code
  * @brief    FLASH Error Code 
  * @{
  */
    /* !< No error                      */
    /* !< Programming Sequence error    */
    /* !< Programming Parallelism error */
    /* !< Programming Alignment error   */
    /* !< Write protection error        */
    /* !< Operation Error               */
    /* !< Read Protection Error         */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Type_Program FLASH Type Program
  * @{
  */
    /* !< Program byte (8-bit) at a specified address           */
    /* !< Program a half-word (16-bit) at a specified address   */
    /* !< Program a word (32-bit) at a specified address        */
    /* !< Program a double word (64-bit) at a specified address */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Flag_definition FLASH Flag definition
  * @brief Flag definition
  * @{
  */
    /* !< FLASH End of Operation flag               */
    /* !< FLASH operation Error flag                */
    /* !< FLASH Write protected error flag          */
    /* !< FLASH Programming Alignment error flag    */
    /* !< FLASH Programming Parallelism error flag  */
    /* !< FLASH Erasing Sequence error flag         */
    /* !< FLASH Busy flag                           */
    /* FLASH_OPTCR2_PCROP */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Interrupt_definition FLASH Interrupt definition
  * @brief FLASH Interrupt definition
  * @{
  */
    /* !< End of FLASH Operation Interrupt source */
    /* !< Error Interrupt source                  */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Program_Parallelism FLASH Program Parallelism
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Keys FLASH Keys
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Sectors FLASH Sectors
  * @{
  */
    /* !< Sector Number 0   */
    /* !< Sector Number 1   */
    /* !< Sector Number 2   */
    /* !< Sector Number 3   */
    /* !< Sector Number 4   */
    /* !< Sector Number 5   */
    /* !< Sector Number 6   */
    /* !< Sector Number 7   */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup FLASH_Exported_Macros FLASH Exported Macros
  * @{
  */
/* *
  * @brief  Set the FLASH Latency.
  * @param  __LATENCY__: FLASH Latency                   
  *         The value of this parameter depend on device used within the same series
  * @retval none
  */
    /* *
  * @brief  Get the FLASH Latency.
  * @retval FLASH Latency                   
  *          The value of this parameter depend on device used within the same series
  */
    /* *
  * @brief  Enable the FLASH prefetch buffer.
  * @retval none
  */
    /* *
  * @brief  Disable the FLASH prefetch buffer.
  * @retval none
  */
    /* *
  * @brief  Enable the FLASH Adaptive Real-Time memory accelerator.
  * @note   The ART accelerator is available only for flash access on ITCM interface.
  * @retval none
  */
    /* *
  * @brief  Disable the FLASH Adaptive Real-Time memory accelerator.
  * @retval none
  */
    /* *
  * @brief  Resets the FLASH Adaptive Real-Time memory accelerator.
  * @note   This function must be used only when the Adaptive Real-Time memory accelerator
  *         is disabled.  
  * @retval None
  */
    /* *
  * @brief  Enable the specified FLASH interrupt.
  * @param  __INTERRUPT__ : FLASH interrupt 
  *         This parameter can be any combination of the following values:
  *     @arg FLASH_IT_EOP: End of FLASH Operation Interrupt
  *     @arg FLASH_IT_ERR: Error Interrupt    
  * @retval none
  */
    /* *
  * @brief  Disable the specified FLASH interrupt.
  * @param  __INTERRUPT__ : FLASH interrupt 
  *         This parameter can be any combination of the following values:
  *     @arg FLASH_IT_EOP: End of FLASH Operation Interrupt
  *     @arg FLASH_IT_ERR: Error Interrupt    
  * @retval none
  */
    /* *
  * @brief  Get the specified FLASH flag status. 
  * @param  __FLAG__: specifies the FLASH flag to check.
  *          This parameter can be one of the following values:
  *            @arg FLASH_FLAG_EOP   : FLASH End of Operation flag 
  *            @arg FLASH_FLAG_OPERR : FLASH operation Error flag 
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag 
  *            @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_ERSERR : FLASH Erasing Sequence error flag 
  *            @arg FLASH_FLAG_BSY   : FLASH Busy flag
  * @retval The new state of __FLAG__ (SET or RESET).
  */
    /* *
  * @brief  Clear the specified FLASH flag.
  * @param  __FLAG__: specifies the FLASH flags to clear.
  *          This parameter can be any combination of the following values:
  *            @arg FLASH_FLAG_EOP   : FLASH End of Operation flag 
  *            @arg FLASH_FLAG_OPERR : FLASH operation Error flag 
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag 
  *            @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag 
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_ERSERR : FLASH Erasing Sequence error flag    
  * @retval none
  */
    /* *
  * @}
  */
    /* Include FLASH HAL Extension module */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup FLASH_Exported_Functions
  * @{
  */
/* * @addtogroup FLASH_Exported_Functions_Group1
  * @{
  */
/* Program operation functions  ***********************************************/
    #[no_mangle]
    fn HAL_FLASH_Program(TypeProgram: uint32_t, Address: uint32_t,
                         Data: uint64_t) -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* * @addtogroup FLASH_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions  **********************************************/
    #[no_mangle]
    fn HAL_FLASH_Unlock() -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_FLASH_Lock() -> HAL_StatusTypeDef;
    #[no_mangle]
    fn failureMode(mode: failureMode_e);
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
    static mut __config_start: uint8_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
pub type uintptr_t = libc::c_ulong;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_def.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   This file contains HAL common defines, enumeration, macros and 
  *          structures definitions. 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  HAL Status structures definition  
  */
pub type HAL_StatusTypeDef = libc::c_uint;
pub const HAL_TIMEOUT: HAL_StatusTypeDef = 3;
pub const HAL_BUSY: HAL_StatusTypeDef = 2;
pub const HAL_ERROR: HAL_StatusTypeDef = 1;
pub const HAL_OK: HAL_StatusTypeDef = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_flash_ex.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of FLASH HAL Extension module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup FLASHEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/ 
/* * @defgroup FLASHEx_Exported_Types FLASH Exported Types
  * @{
  */
/* *
  * @brief  FLASH Erase structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FLASH_EraseInitTypeDef {
    pub TypeErase: uint32_t,
    pub Sector: uint32_t,
    pub NbSectors: uint32_t,
    pub VoltageRange: uint32_t,
}
pub type failureMode_e = libc::c_uint;
pub const FAILURE_GYRO_INIT_FAILED: failureMode_e = 6;
pub const FAILURE_FLASH_WRITE_FAILED: failureMode_e = 5;
pub const FAILURE_INVALID_EEPROM_CONTENTS: failureMode_e = 4;
pub const FAILURE_ACC_INCOMPATIBLE: failureMode_e = 3;
pub const FAILURE_ACC_INIT: failureMode_e = 2;
pub const FAILURE_MISSING_ACC: failureMode_e = 1;
pub const FAILURE_DEVELOPER: failureMode_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct config_streamer_s {
    pub address: uintptr_t,
    pub size: libc::c_int,
    pub buffer: C2RustUnnamed,
    pub at: libc::c_int,
    pub err: libc::c_int,
    pub unlocked: bool,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub b: [uint8_t; 4],
    pub w: uint32_t,
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
// Streams data out to the EEPROM, padding to the write size as
// needed, and updating the checksum as it goes.
pub type config_streamer_t = config_streamer_s;
#[no_mangle]
pub unsafe extern "C" fn config_streamer_init(mut c: *mut config_streamer_t) {
    memset(c as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<config_streamer_t>() as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn config_streamer_start(mut c: *mut config_streamer_t,
                                               mut base: uintptr_t,
                                               mut size: libc::c_int) {
    // base must start at FLASH_PAGE_SIZE boundary
    (*c).address = base;
    (*c).size = size;
    if !(*c).unlocked {
        HAL_FLASH_Unlock();
        (*c).unlocked = 1 as libc::c_int != 0
    }
    // NOP
    (*c).err = 0 as libc::c_int;
}
/*
Sector 0    0x08000000 - 0x08007FFF 32 Kbytes
Sector 1    0x08008000 - 0x0800FFFF 32 Kbytes
Sector 2    0x08010000 - 0x08017FFF 32 Kbytes
Sector 3    0x08018000 - 0x0801FFFF 32 Kbytes
Sector 4    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 5    0x08040000 - 0x0807FFFF 256 Kbytes
Sector 6    0x08080000 - 0x080BFFFF 256 Kbytes
Sector 7    0x080C0000 - 0x080FFFFF 256 Kbytes
*/
unsafe extern "C" fn getFLASHSectorForEEPROM() -> uint32_t {
    if &mut __config_start as *mut uint8_t as uint32_t <=
           0x8007fff as libc::c_int as libc::c_uint {
        return 0 as libc::c_uint
    }
    if &mut __config_start as *mut uint8_t as uint32_t <=
           0x800ffff as libc::c_int as libc::c_uint {
        return 1 as libc::c_uint
    }
    if &mut __config_start as *mut uint8_t as uint32_t <=
           0x8017fff as libc::c_int as libc::c_uint {
        return 2 as libc::c_uint
    }
    if &mut __config_start as *mut uint8_t as uint32_t <=
           0x801ffff as libc::c_int as libc::c_uint {
        return 3 as libc::c_uint
    }
    if &mut __config_start as *mut uint8_t as uint32_t <=
           0x803ffff as libc::c_int as libc::c_uint {
        return 4 as libc::c_uint
    }
    if &mut __config_start as *mut uint8_t as uint32_t <=
           0x807ffff as libc::c_int as libc::c_uint {
        return 5 as libc::c_uint
    }
    if &mut __config_start as *mut uint8_t as uint32_t <=
           0x80bffff as libc::c_int as libc::c_uint {
        return 6 as libc::c_uint
    }
    if &mut __config_start as *mut uint8_t as uint32_t <=
           0x80fffff as libc::c_int as libc::c_uint {
        return 7 as libc::c_uint
    }
    loop 
         // Not good
         {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
    };
}
unsafe extern "C" fn write_word(mut c: *mut config_streamer_t,
                                mut value: uint32_t) -> libc::c_int {
    if (*c).err != 0 as libc::c_int { return (*c).err }
    if (*c).address.wrapping_rem(0x8000 as libc::c_int as uint32_t as
                                     libc::c_ulong) ==
           0 as libc::c_int as libc::c_ulong {
        let mut EraseInitStruct: FLASH_EraseInitTypeDef =
            {
                let mut init =
                    FLASH_EraseInitTypeDef{TypeErase: 0 as libc::c_uint,
                                           Sector: 0,
                                           NbSectors:
                                               1 as libc::c_int as uint32_t,
                                           VoltageRange:
                                               0x2 as libc::c_uint,};
                init
            };
        EraseInitStruct.Sector = getFLASHSectorForEEPROM();
        let mut SECTORError: uint32_t = 0;
        let status: HAL_StatusTypeDef =
            HAL_FLASHEx_Erase(&mut EraseInitStruct, &mut SECTORError);
        if status as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            return -(1 as libc::c_int)
        }
    }
    let status_0: HAL_StatusTypeDef =
        HAL_FLASH_Program(0x2 as libc::c_uint, (*c).address as uint32_t,
                          value as uint64_t);
    if status_0 as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        return -(2 as libc::c_int)
    }
    (*c).address =
        ((*c).address as
             libc::c_ulong).wrapping_add(::core::mem::size_of::<uint32_t>() as
                                             libc::c_ulong) as uintptr_t as
            uintptr_t;
    return 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn config_streamer_write(mut c: *mut config_streamer_t,
                                               mut p: *const uint8_t,
                                               mut size: uint32_t)
 -> libc::c_int {
    let mut pat: *const uint8_t = p;
    while pat != (p as *mut uint8_t).offset(size as isize) as *const uint8_t {
        let fresh0 = (*c).at;
        (*c).at = (*c).at + 1;
        (*c).buffer.b[fresh0 as usize] = *pat;
        if (*c).at as libc::c_ulong ==
               ::core::mem::size_of::<C2RustUnnamed>() as libc::c_ulong {
            (*c).err = write_word(c, (*c).buffer.w);
            (*c).at = 0 as libc::c_int
        }
        pat = pat.offset(1)
    }
    return (*c).err;
}
#[no_mangle]
pub unsafe extern "C" fn config_streamer_status(mut c: *mut config_streamer_t)
 -> libc::c_int {
    return (*c).err;
}
#[no_mangle]
pub unsafe extern "C" fn config_streamer_flush(mut c: *mut config_streamer_t)
 -> libc::c_int {
    if (*c).at != 0 as libc::c_int {
        memset((*c).buffer.b.as_mut_ptr().offset((*c).at as isize) as
                   *mut libc::c_void, 0 as libc::c_int,
               (::core::mem::size_of::<C2RustUnnamed>() as
                    libc::c_ulong).wrapping_sub((*c).at as libc::c_ulong));
        (*c).err = write_word(c, (*c).buffer.w);
        (*c).at = 0 as libc::c_int
    }
    return (*c).err;
}
#[no_mangle]
pub unsafe extern "C" fn config_streamer_finish(mut c: *mut config_streamer_t)
 -> libc::c_int {
    if (*c).unlocked {
        HAL_FLASH_Lock();
        (*c).unlocked = 0 as libc::c_int != 0
    }
    return (*c).err;
}
